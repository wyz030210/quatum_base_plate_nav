"""Unified request-scoped start/cancel/status API and single active lifecycle."""
import copy
import json
import math
from datetime import datetime, timezone
import time
import uuid
from pathlib import Path

from .single_point_inspection import SinglePointInspection
from .task_adapters import ExecutionUpdate, StopState


class TaskManager:
    """Serial, nonblocking control API; call tick from the same executor as requests.

    Inject system boundary adapters. Handlers may extend task types without owning
    authorization, stop confirmation, task identity or report persistence.
    """

    def __init__(self, navigation, localization, stop, report_dir, *, clock=time.monotonic,
                 handlers=None, state_timeout=1.0, cancel_timeout=5.0, task_timeout=600.0):
        self.localization = localization
        self.stop = stop
        self.report_dir = Path(report_dir)
        self.clock = clock
        self.handlers = handlers or {'inspection': SinglePointInspection(navigation)}
        for value in (state_timeout, cancel_timeout, task_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('task timeouts must be finite and positive')
        self.state_timeout = state_timeout
        self.cancel_timeout = cancel_timeout
        self.task_timeout = task_timeout
        self.current = None
        self.operation = None
        self.active = False
        self.stopping_at = None
        self.authorizing_at = None
        self.desired_result = None

    def start(self, task_type, request):
        if self.active:
            return {'accepted': False, 'task_id': '', 'message': 'task active or stop unconfirmed'}
        if task_type not in self.handlers:
            return {'accepted': False, 'task_id': '', 'message': 'unsupported task type'}
        try:
            config = self.handlers[task_type].prepare(request)
        except (ValueError, TypeError, KeyError) as exc:
            return {'accepted': False, 'task_id': '', 'message': str(exc)}
        now = self.clock()
        loc_error = self._localization_error(config, now)
        safety = self._stop_state(now)
        if (loc_error or not self._safety_fresh(safety, now) or
                not self._safety_permits(safety) or not safety.stopped or
                not safety.stop_requested):
            return {'accepted': False, 'task_id': '',
                    'message': loc_error or safety.fault or 'stop layer not ready'}
        task_id = uuid.uuid4().hex
        self.current = {'task_id': task_id, 'task_type': task_type, 'config': config,
                        'phase': 'dependency_check', 'result': None, 'error': '',
                        'report_error': '', 'report_path': '',
                        'simulated': task_type == 'inspection',
                        'started_at': now, 'finished_at': None, 'details': {},
                        'started_utc': datetime.now(timezone.utc).isoformat(),
                        'cleanup_confirmed': False, 'phases': ['dependency_check']}
        self.active = True
        self.operation = None
        self.stopping_at = None
        self.authorizing_at = now
        self.desired_result = None
        try:
            self.stop.authorize(task_id)
        except Exception as exc:
            self._stop('failed', f'authorization transport error: {exc}')
        return {'accepted': True, 'task_id': task_id, 'message': 'accepted; waiting authorization'}

    def shutdown(self):
        if self.active:
            self._stop('failed', 'supervisor shutdown')

    def cancel(self, task_id=''):
        if not self.active:
            return {'accepted': True, 'task_id': task_id, 'message': 'no active task'}
        if task_id and task_id != self.current['task_id']:
            return {'accepted': False, 'task_id': task_id,
                    'message': 'task id does not match active task'}
        self._stop('canceled', 'user canceled')
        return {'accepted': True, 'task_id': self.current['task_id'],
                'message': 'cancel accepted; stop completion pending'}

    def status(self):
        if self.current is None:
            return {'task_id': '', 'phase': 'idle', 'result': None, 'active': False}
        return copy.deepcopy(dict(self.current, active=self.active))

    def _phase(self, phase):
        if self.current['phase'] != phase:
            self.current['phase'] = phase
            self.current['phases'].append(phase)

    def _stop_state(self, now):
        try:
            return self.stop.snapshot(now)
        except Exception as exc:
            return StopState(False, False, True, now, None,
                             fault=f'stop transport error: {exc}')

    def _localization_error(self, config, now):
        try:
            loc = self.localization.snapshot(now)
        except Exception as exc:
            return f'localization transport error: {exc}'
        if (not loc.valid or loc.frame_id != config['frame_id'] or
                not 0 <= now - loc.received_at <= self.state_timeout or
                not 0 <= loc.source_age_sec <= self.state_timeout):
            return loc.error or 'localization invalid, stale or frame mismatch'
        return ''

    def _safety_fresh(self, safety, now):
        return (0 <= now - safety.received_at <= self.state_timeout and
                safety.feedback_age_sec is not None and
                0 <= safety.feedback_age_sec <= self.state_timeout)

    def _safety_permits(self, safety):
        return (safety.ready and not safety.manual_override and
                not safety.emergency_stop and not safety.fault)

    def _stop(self, result, error=''):
        if self.stopping_at is not None:
            return
        self.desired_result = result
        self.current['error'] = error
        self.stopping_at = self.clock()
        self._phase('stopping')
        try:
            self.stop.request_stop(self.current['task_id'])
        except Exception as exc:
            self.current['cleanup_error'] = f'stop request transport error: {exc}'
        if self.operation:
            try:
                self.operation.cancel()
            except Exception as exc:
                self.current['cleanup_error'] = f'cancel transport error: {exc}'

    def _poll(self, now):
        try:
            return self.operation.poll(now)
        except Exception as exc:
            return ExecutionUpdate('executing', error=f'execution transport error: {exc}')

    def tick(self):
        if not self.active:
            return
        now = self.clock()
        safety = self._stop_state(now)
        if self.stopping_at is None:
            error = self._localization_error(self.current['config'], now)
            if error:
                self._stop('failed', error)
            elif not self._safety_fresh(safety, now) or not self._safety_permits(safety):
                self._stop('failed', safety.fault or 'safety invalid or manual/emergency stop')
            elif self.operation is not None and safety.stop_requested:
                self._stop('failed', 'external stop requested')
            elif now - self.current['started_at'] > self.task_timeout:
                self._stop('failed', 'task_timeout')
        if self.stopping_at is not None:
            update = None if self.operation is None else self._poll(now)
            done = update is None or update.done
            if update is not None and update.error and not update.done:
                self.current['cleanup_error'] = update.error
            if (done and safety.stopped and safety.stop_requested and
                    self._safety_fresh(safety, now) and safety.received_at > self.stopping_at):
                self._finish(now)
            elif now - self.stopping_at > self.cancel_timeout:
                self.current['cleanup_error'] = (
                    'cancel_timeout: waiting owned action terminal and fresh stop')
            return
        if self.operation is None:
            if safety.stop_requested or safety.received_at < self.authorizing_at:
                return
            try:
                self.operation = self.handlers[self.current['task_type']].begin(
                    self.current['task_id'], self.current['config'], now)
            except Exception as exc:
                self._stop('failed', f'execution start failed: {exc}')
                return
        update = self._poll(now)
        self._phase(update.phase)
        if update.details:
            self.current['details'] = copy.deepcopy(update.details)
        if update.error and not update.done:
            self._stop('failed', update.error)
        elif update.done:
            self._stop('succeeded' if update.succeeded else 'failed', update.error)

    def _finish(self, now):
        self.current['result'] = self.desired_result
        self.current['elapsed_sec'] = max(0.0, now - self.current['started_at'])
        self.current['finished_at'] = now
        self.current['finished_utc'] = datetime.now(timezone.utc).isoformat()
        self.current['execution_result'] = self.desired_result
        self.current['cleanup_confirmed'] = True
        self._phase('finished')
        try:
            self.report_dir.mkdir(parents=True, exist_ok=True)
            path = self.report_dir / (self.current['task_id'] + '.json')
            self.current['report_path'] = str(path)
            payload = json.dumps(self.current, ensure_ascii=False, allow_nan=False, indent=2)
            temporary = path.with_suffix('.json.tmp')
            temporary.write_text(payload + '\n', encoding='utf-8')
            temporary.replace(path)
        except (OSError, ValueError) as exc:
            self.current['report_path'] = ''
            self.current['report_error'] = str(exc)
            self.current['result'] = 'failed'
        self.active = False

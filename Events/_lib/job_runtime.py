from java.lang import Runnable, Thread
from java.util.concurrent.atomic import AtomicBoolean, AtomicReference


class JobStopRequested(BaseException):
    def __init__(self, message):
        BaseException.__init__(self, message)
        self.cleanup_errors = []


class GuiJobStateApi(object):
    def __init__(self, gui_obj, optional_script_api=None):
        self.gui = gui_obj
        self.optional_script_api = optional_script_api

    def job_tab(self):
        tab = self.gui.getJobTab()
        if tab is None:
            raise Exception("OpenPnP Job tab is unavailable.")
        return tab

    def get(self):
        return self.job_tab().getJobState()

    def isPauseRequested(self):
        return self.job_tab().isPauseRequested()

    def pauseIfRequested(self):
        return self.job_tab().pauseIfRequested()

    def cancel(self):
        if (self.optional_script_api is not None and
                hasattr(self.optional_script_api, "cancel")):
            return self.optional_script_api.cancel()
        raise Exception("Job stopped by user after safe cleanup.")


class JobRuntimeMonitor(Runnable):
    def __init__(self, state_api, tag, log_fn, log_err_fn, poll_ms=50):
        self.state_api = state_api
        self.tag = tag
        self.log = log_fn
        self.log_err = log_err_fn
        self.poll_ms = int(poll_ms)
        self.state = AtomicReference("Running")
        self.shutdown_requested = AtomicBoolean(False)
        self.cleanup_started = AtomicBoolean(False)
        self.thread = Thread(self, "%s job-state-monitor" % tag)
        self.thread.setDaemon(True)

    def start(self):
        initial_state = str(self.state_api.get())
        self.state.set(initial_state)
        self.log("Job-state monitor starting in state %s." % initial_state)
        self.thread.start()

    def run(self):
        previous = self.state.get()
        while not self.shutdown_requested.get():
            try:
                current = str(self.state_api.get())
            except Exception, e:
                self.log_err(
                    "Job-state read failed; requesting safe stop: %s" % e)
                current = "Stopped"
            self.state.set(current)
            if current != previous:
                self.log("Job state changed: %s -> %s" %
                         (previous, current))
                previous = current
            Thread.sleep(self.poll_ms)

    def begin_cleanup(self):
        self.cleanup_started.set(True)

    def checkpoint(self, label, allow_pause=True):
        if self.cleanup_started.get():
            return

        if allow_pause and bool(self.state_api.isPauseRequested()):
            self.log("Reached pause checkpoint: %s" % label)
            self.state_api.pauseIfRequested()
            current = str(self.state_api.get())
            self.state.set(current)
            if current == "Running":
                self.log("Resuming script at checkpoint: %s" % label)

        current = str(self.state_api.get())
        self.state.set(current)
        if current in ("Stopping", "Stopped"):
            raise JobStopRequested(
                "Job entered %s at checkpoint: %s" % (current, label))
        if current not in ("Running", "Pausing", "Paused"):
            raise Exception("Unexpected job state '%s' at checkpoint: %s" %
                            (current, label))

    def stop(self):
        self.shutdown_requested.set(True)
        if Thread.currentThread() is not self.thread:
            self.thread.join(self.poll_ms * 4)
        self.log("Job-state monitor stopped.")

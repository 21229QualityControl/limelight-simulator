// Pipeline delay.
//
// The controller never sees the present. On the real robot a detection is
// about 34 ms old by the time Java reads it -- capture plus the SnapScript's
// processing -- and the camera is polled at 20 Hz, so it can be up to 50 ms
// older still before a fresh one arrives. The robot keeps moving through all
// of that, so the bearing it steers on describes where the ball WAS.
//
// This matters because delay destabilises a proportional loop. A gain that
// is calm with no delay will hunt with 80 ms of it, and the sim would happily
// bless gains that oscillate on the real robot. Feeding detections instantly
// is the most optimistic thing a simulator can do.

export function defaultPipeline() {
  return {
    // Capture + processing, measured on the real camera: 13 ms with an empty
    // frame, 34 ms with a crowded one. Latency rises exactly when the field
    // is busiest.
    latencyMs: 34,
    // limelight.setPollRateHz(20) in LimelightVision.
    pollHz: 20,
  };
}

/** Holds detections until they are old enough to be delivered. */
export class DelayLine {
  constructor(pipeline = defaultPipeline()) {
    this.p = pipeline;
    this.queue = [];
    this.delivered = null;
    this.lastPollMs = -1e9;
  }

  reset() {
    this.queue = [];
    this.delivered = null;
    this.lastPollMs = -1e9;
  }

  /** Record what the camera saw at this instant. */
  push(nowMs, detection) {
    this.queue.push({ t: nowMs, detection });
    if (this.queue.length > 256) this.queue.shift();
  }

  /**
   * What the controller is allowed to know right now: the most recent
   * detection that is at least latencyMs old, and only if the poll interval
   * has elapsed since the last one was handed over.
   */
  read(nowMs) {
    const pollInterval = 1000 / this.p.pollHz;
    if (nowMs - this.lastPollMs < pollInterval) return this.delivered;

    let found;
    while (this.queue.length && nowMs - this.queue[0].t >= this.p.latencyMs) {
      found = this.queue.shift();
    }
    if (found) {
      this.delivered = found.detection;
      this.lastPollMs = nowMs;
    }
    return this.delivered;
  }

  /** Age of the detection currently being acted on, ms. */
  ageMs(nowMs) {
    return nowMs - this.lastPollMs + this.p.latencyMs;
  }
}

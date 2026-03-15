from drobot_tracker.kalman_filter import KalmanFilter6D


class Track:
    """
    Single object track with lifecycle: created → confirmed → lost → deleted.

    Lifecycle:
      - Created with first detection
      - Confirmed after min_hits consecutive hits
      - Lost after consecutive misses
      - Deleted after max_misses consecutive misses
    """

    _next_id = 0

    @classmethod
    def reset_id_counter(cls):
        cls._next_id = 0

    def __init__(self, position, label='', confidence=0.0,
                 process_noise_std=0.5, measurement_noise_std=0.3):
        self.track_id = Track._next_id
        Track._next_id += 1

        self.label = label
        self.confidence = confidence

        self.kf = KalmanFilter6D(
            initial_pos=position,
            process_noise_std=process_noise_std,
            measurement_noise_std=measurement_noise_std,
        )

        self.consecutive_hits = 1
        self.consecutive_misses = 0
        self.total_hits = 1
        self.age = 0  # number of frames since creation

    def predict(self, dt):
        """Predict next state. Called every frame."""
        self.kf.predict(dt)
        self.age += 1

    def update(self, measurement, label='', confidence=0.0):
        """Update with new detection."""
        self.kf.update(measurement)
        self.consecutive_hits += 1
        self.total_hits += 1
        self.consecutive_misses = 0
        if confidence > self.confidence:
            self.confidence = confidence
            self.label = label

    def mark_missed(self):
        """No matching detection this frame."""
        self.consecutive_misses += 1
        self.consecutive_hits = 0

    def is_confirmed(self, min_hits=3):
        return self.total_hits >= min_hits

    def is_dead(self, max_misses=5):
        return self.consecutive_misses > max_misses

    @property
    def predicted_position(self):
        return self.kf.position

    @property
    def velocity(self):
        return self.kf.velocity

    @property
    def position(self):
        return self.kf.position

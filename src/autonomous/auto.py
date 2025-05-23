from autonomous.auto_base import AutoBase


class test_L(AutoBase):
    MODE_NAME = "testL"

    def __init__(self):
        super().__init__(["test-testl"])

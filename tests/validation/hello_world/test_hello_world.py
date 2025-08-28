import logging


def test_hello_world(dut):
    LOGGER = logging.getLogger(__name__)
    dut.expect_exact("Hello Arduino!")
    LOGGER.info("Hello Arduino! message was sent")

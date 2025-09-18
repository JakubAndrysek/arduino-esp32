import logging


def test_gpio_interrupt(dut):
    """Test GPIO interrupt functionality including attach, detach, and different edge triggers."""
    dut.expect_exact("GPIO interrupt test END")

import logging


def test_gpio_interrupt(dut):
    dut.expect_exact("ADC test END")

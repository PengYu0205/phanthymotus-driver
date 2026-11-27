"""Motus MCP entry point for the PrimeBot Q1 Explorer."""
from common.vendor_runtime import run_driver
from battery import BatteryCard
from expression import ExpressionCard


def build_plugins(config, namespace, ros):
    robot_domain = config["ros"]["robot_domain_id"]
    return [BatteryCard(namespace, ros, robot_domain),
            ExpressionCard(namespace, ros, robot_domain)]


if __name__ == "__main__":
    run_driver(__file__, "primebot-q1-driver", "primebot-q1-device-bundle", build_plugins)

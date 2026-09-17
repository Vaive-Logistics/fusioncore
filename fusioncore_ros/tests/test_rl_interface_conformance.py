# Substitutability contract: where FusionCore answers to a robot_localization
# interface name, it must answer with robot_localization's TYPE.
#
# This generalises the bug in issue #73. /fromLL was advertised as
# fusioncore_ros/srv/FromLL, whose fields are identical to
# robot_localization/srv/FromLL. ROS 2 matches services on name AND type, so
# nav2_waypoint_follower, which has the robot_localization type compiled in, waited
# forever for a service it could not see. It never errored. It shipped that way from
# v0.2.1 to 0.3.4 and was only found because one user was persistent enough to keep
# filing.
#
# test_from_ll_service.py pins that one service. This pins the CLASS of bug: right
# name, wrong type. A future service added under a robot_localization name with a
# locally-defined type would reproduce it exactly, and would again fail silently by
# hanging rather than erroring.
#
# It also pins the NEGATIVE half. Services robot_localization has and FusionCore does
# not are listed explicitly and asserted absent. That is deliberate: a half-finished
# ToLL advertised under the right name with the wrong type is worse than no ToLL,
# because absent fails fast and wrong-type hangs. If you implement one, import the
# robot_localization type and move it to the implemented list in the same commit.

import os
import unittest

os.environ.setdefault('ROS_DOMAIN_ID', str(42 + os.getpid() % 40))

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState

# Services FusionCore deliberately provides under a robot_localization name. The
# value is the type it MUST advertise, not the type that would merely compile.
IMPLEMENTED = {
    '/fromLL': 'robot_localization/srv/FromLL',
}

# Services robot_localization has that FusionCore does not. Asserted ABSENT so a
# partial implementation cannot land under the right name with the wrong type.
#
# Worth knowing if you are migrating: ToLL is the inverse of fromLL, SetPose is how
# robot_localization is told to re-initialise (FusionCore uses a std_srvs/Trigger
# reset instead, which takes no pose), and SetDatum fixes the GNSS origin (FusionCore
# uses the reference.* parameters).
NOT_IMPLEMENTED = ['/toLL', '/set_pose', '/set_datum', '/setUTMZone',
                   '/toggle_filter_processing', '/fromLLArray']


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    node = launch_ros.actions.Node(
        package='fusioncore_ros', executable='fusioncore_node', name='fusioncore',
        output='screen',
        parameters=[{'init.wait_for_all_sensors': False,
                     'reference.use_first_fix': False,
                     'reference.x': 814818.4192,
                     'reference.y': -4580454.6512,
                     'reference.z': 4348559.7245}])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestRlInterfaceConformance(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_rl_conformance')
        cli = cls.node.create_client(ChangeState, '/fusioncore/change_state')
        assert cli.wait_for_service(timeout_sec=30.0), 'lifecycle service never appeared'
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(cls.node, fut, timeout_sec=30.0)
        assert fut.result() is not None, 'configure transition timed out'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _services(self, timeout=30.0):
        """Wait until every implemented name is visible, then snapshot the graph."""
        import time
        end = time.time() + timeout
        found = {}
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            found = dict(self.node.get_service_names_and_types())
            if all(n in found for n in IMPLEMENTED):
                return found
        return found

    def test_implemented_services_use_the_robot_localization_type(self):
        found = self._services()
        for name, want in IMPLEMENTED.items():
            self.assertIn(name, found,
                          f'{name} is not advertised at all. Nav2 and any other client '
                          f'holding the type will hang rather than error.')
            self.assertIn(
                want, found[name],
                f'{name} advertises {found[name]} but must advertise {want}. '
                f'ROS 2 matches services on name AND type, so a client compiled '
                f'against {want} cannot see this service and will wait forever. '
                f'Identical fields under a different type name is not compatibility. '
                f'This is issue #73 happening again.')

    def test_unimplemented_services_are_absent_rather_than_wrong(self):
        found = self._services()
        for name in NOT_IMPLEMENTED:
            self.assertNotIn(
                name, found,
                f'{name} has appeared. If that is deliberate, it must advertise the '
                f'robot_localization type and move to IMPLEMENTED in this file. '
                f'Absent fails fast; present-with-the-wrong-type hangs the client, '
                f'which is strictly worse and is what issue #73 cost.')

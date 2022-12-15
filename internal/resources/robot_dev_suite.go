package resources

import (
	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	"k8s.io/apimachinery/pkg/types"
)

func GetRobotVDI(robotDevSuite *robotv1alpha1.RobotDevSuite, robotVDINamespacedName *types.NamespacedName) *robotv1alpha1.RobotVDI {
	return &robotv1alpha1.RobotVDI{}
}

func GetRobotIDE(robotDevSuite *robotv1alpha1.RobotDevSuite, robotIDENamespacedName *types.NamespacedName) *robotv1alpha1.RobotIDE {
	return &robotv1alpha1.RobotIDE{}
}

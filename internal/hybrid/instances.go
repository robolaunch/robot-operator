package hybrid

import (
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

func HasLaunchInThisInstance(launchManager robotv1alpha1.LaunchManager, robot robotv1alpha1.Robot) bool {

	clusterName := label.GetClusterName(&robot)

	for _, l := range launchManager.Spec.Launch {
		if ContainsInstance(l.Instances, clusterName) {
			return true
		}
	}

	return false
}

func ContainsInstance(instances []string, instance string) bool {

	if len(instances) == 0 {
		return true
	}

	for _, v := range instances {
		if v == instance {
			return true
		}
	}

	return false
}

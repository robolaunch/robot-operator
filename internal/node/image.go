package node

import (
	"path/filepath"
	"strings"

	"github.com/robolaunch/robot-operator/internal"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

// Not used in robot manifest, needed for internal use.
type ReadyRobotProperties struct {
	Enabled bool
	Image   string
}

func GetReadyRobotProperties(robot robotv1alpha1.Robot) ReadyRobotProperties {
	labels := robot.GetLabels()

	if user, hasUser := labels[internal.ROBOT_IMAGE_USER]; hasUser {
		if repository, hasRepository := labels[internal.ROBOT_IMAGE_REPOSITORY]; hasRepository {
			if tag, hasTag := labels[internal.ROBOT_IMAGE_TAG]; hasTag {
				return ReadyRobotProperties{
					Enabled: true,
					Image:   user + "/" + repository + ":" + tag,
				}
			}
		}
	}

	return ReadyRobotProperties{
		Enabled: false,
	}
}

func GetImage(node corev1.Node, robot robotv1alpha1.Robot) string {

	var imageBuilder strings.Builder
	var tagBuilder strings.Builder

	distributions := robot.Spec.Distributions
	readyRobot := GetReadyRobotProperties(robot)

	if readyRobot.Enabled {

		imageBuilder.WriteString(readyRobot.Image)

	} else {

		registry := "robolaunchio"
		repository := "robot"

		tagBuilder.WriteString("base-")
		tagBuilder.WriteString(getDistroStr(distributions))

		hasGPU := HasGPU(node)

		if hasGPU {

			tagBuilder.WriteString("-agnostic-")
			tagBuilder.WriteString("xfce") // TODO: make desktop selectable

		} else {
			tagBuilder.WriteString("-agnostic-")
			tagBuilder.WriteString("xfce") // TODO: make desktop selectable
		}

		imageBuilder.WriteString(filepath.Join(registry, repository) + ":")
		imageBuilder.WriteString(tagBuilder.String())

	}

	return imageBuilder.String()

}

func getDistroStr(distributions []robotv1alpha1.ROSDistro) string {

	if len(distributions) == 1 {
		return string(distributions[0])
	}

	return setPrecisionBetweenDistributions(distributions)
}

func setPrecisionBetweenDistributions(distributions []robotv1alpha1.ROSDistro) string {

	if distributions[0] == robotv1alpha1.ROSDistroFoxy || distributions[1] == robotv1alpha1.ROSDistroFoxy {
		return string(robotv1alpha1.ROSDistroFoxy) + "-" + string(robotv1alpha1.ROSDistroGalactic)
	}

	// TODO: Add validation webhook for distro selections
	// multi-distro is allowed for only foxy & galactic
	return ""
}

package node

import (
	"path/filepath"
	"strings"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	"github.com/robolaunch/robot-operator/internal"
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

	distro := robot.Spec.Distro
	readyRobot := GetReadyRobotProperties(robot)

	if readyRobot.Enabled {

		imageBuilder.WriteString(readyRobot.Image)

	} else {

		registry := "robolaunchio"
		repository := "robot"

		tagBuilder.WriteString("base-")
		tagBuilder.WriteString(string(distro))

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

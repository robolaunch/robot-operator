package node

import (
	"errors"
	"io"
	"net/http"
	"path/filepath"
	"strings"

	"github.com/robolaunch/robot-operator/internal"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"gopkg.in/yaml.v2"
	corev1 "k8s.io/api/core/v1"
)

type Platform struct {
	Versions []Version `yaml:"versions"`
}

type Version struct {
	Date    string `yaml:"date"`
	Version string `yaml:"version"`
	Images  Images `yaml:"images"`
}

type Images struct {
	Organization string  `yaml:"organization"`
	Repository   string  `yaml:"repository"`
	Domains      Domains `yaml:"domains"`
}

type Domains struct {
	Robotics []Element `yaml:"robotics"`
}

type Element struct {
	Application   Application   `yaml:"application"`
	DevSpaceImage DevSpaceImage `yaml:"devspace"`
}

type Application struct {
	Name    string `yaml:"name"`
	Version string `yaml:"version"`
}

type DevSpaceImage struct {
	UbuntuDistro string `yaml:"ubuntuDistro"`
	Desktop      string `yaml:"desktop"`
	Version      string `yaml:"version"`
}

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

// TODO: Fetch all properties from platform versioning map
func GetImage(node corev1.Node, robot robotv1alpha1.Robot) (string, error) {

	var imageBuilder strings.Builder
	var tagBuilder strings.Builder

	readyRobot := GetReadyRobotProperties(robot)

	if readyRobot.Enabled {

		imageBuilder.WriteString(readyRobot.Image)

	} else {

		platformVersion := GetPlatformVersion(node)
		imageProps, err := getImageProps(platformVersion, getDistroStr(robot.Spec.Distributions))
		if err != nil {
			return "", err
		}

		organization := "robolaunchio"
		repository := "devspace-robotics"
		tagBuilder.WriteString(imageProps.Application.Name + "-")
		tagBuilder.WriteString(imageProps.Application.Version)
		tagBuilder.WriteString("-" + imageProps.DevSpaceImage.UbuntuDistro + "-" + imageProps.DevSpaceImage.Desktop)
		tagBuilder.WriteString("-" + imageProps.DevSpaceImage.Version)
		imageBuilder.WriteString(filepath.Join(organization, repository) + ":")
		imageBuilder.WriteString(tagBuilder.String())

	}

	return imageBuilder.String(), nil

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
	return ""
}

func getImageProps(platformVersion, distro string) (Element, error) {

	resp, err := http.Get("https://raw.githubusercontent.com/robolaunch/robolaunch/main/platform.yaml")
	if err != nil {
		return Element{}, err
	}

	defer resp.Body.Close()

	var yamlFile []byte
	if resp.StatusCode == http.StatusOK {
		yamlFile, err = io.ReadAll(resp.Body)
		if err != nil {
			return Element{}, err
		}
	}

	var platform Platform
	err = yaml.Unmarshal(yamlFile, &platform)
	if err != nil {
		return Element{}, err
	}

	distroFound := false
	var imageProps Element
	for _, v := range platform.Versions {
		if v.Version == platformVersion {
			for _, element := range v.Images.Domains.Robotics {
				if element.Application.Version == distro {
					imageProps = element
					distroFound = true
					break
				}
			}
		}
	}

	if !distroFound {
		return Element{}, errors.New("distro not found in platform versioning map")
	}

	return imageProps, nil
}

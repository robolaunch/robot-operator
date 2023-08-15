package node

import (
	"errors"
	"io"
	"net/http"
	"path/filepath"
	"reflect"
	"strings"

	cosmodrome "github.com/robolaunch/cosmodrome/pkg/api"
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
	Organization string               `yaml:"organization"`
	Repository   string               `yaml:"repository"`
	Domains      map[string][]Element `yaml:"domains"`
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

	if registry, hasRegistry := labels[internal.ROBOT_IMAGE_REGISTRY_LABEL_KEY]; hasRegistry {
		if user, hasUser := labels[internal.ROBOT_IMAGE_USER_LABEL_KEY]; hasUser {
			if repository, hasRepository := labels[internal.ROBOT_IMAGE_REPOSITORY_LABEL_KEY]; hasRepository {
				if tag, hasTag := labels[internal.ROBOT_IMAGE_TAG_LABEL_KEY]; hasTag {
					return ReadyRobotProperties{
						Enabled: true,
						Image:   registry + "/" + user + "/" + repository + ":" + tag,
					}
				}
			}
		}
	}

	return ReadyRobotProperties{
		Enabled: false,
	}
}

func GetImage(node corev1.Node, robot robotv1alpha1.Robot) (string, error) {
	if robot.Spec.Type == robotv1alpha1.TypeRobot {
		return GetImageForRobot(node, robot)
	} else if robot.Spec.Type == robotv1alpha1.TypeEnvironment {
		return GetImageForEnvironment(node, robot)
	}
	return "", errors.New("cannot specify the resource type")
}

func GetImageForRobot(node corev1.Node, robot robotv1alpha1.Robot) (string, error) {
	var imageBuilder strings.Builder
	var tagBuilder strings.Builder

	readyRobot := GetReadyRobotProperties(robot)

	if readyRobot.Enabled {

		imageBuilder.WriteString(readyRobot.Image)

	} else {

		platformVersion := GetPlatformVersion(node)
		imageProps, err := getImagePropsForRobot(platformVersion, getDistroStr(robot.Spec.RobotConfig.Distributions))
		if err != nil {
			return "", err
		}

		registry, hasRegistry := robot.Labels[internal.ROBOT_IMAGE_REGISTRY_LABEL_KEY]
		if !hasRegistry {
			return "", errors.New("registry is not found in label with key " + internal.ROBOT_IMAGE_REGISTRY_LABEL_KEY)
		}

		organization := "robolaunchio"
		repository := "devspace-robotics"
		tagBuilder.WriteString(imageProps.Application.Name + "-")
		tagBuilder.WriteString(imageProps.Application.Version)
		tagBuilder.WriteString("-" + imageProps.DevSpaceImage.UbuntuDistro + "-" + imageProps.DevSpaceImage.Desktop)
		tagBuilder.WriteString("-" + imageProps.DevSpaceImage.Version)
		imageBuilder.WriteString(filepath.Join(registry, organization, repository) + ":")
		imageBuilder.WriteString(tagBuilder.String())

	}

	return imageBuilder.String(), nil
}

func GetImageForEnvironment(node corev1.Node, robot robotv1alpha1.Robot) (string, error) {
	var imageBuilder strings.Builder
	var tagBuilder strings.Builder

	readyEnvironment := GetReadyRobotProperties(robot)

	if readyEnvironment.Enabled {

		imageBuilder.WriteString(readyEnvironment.Image)

	} else {

		platformVersion := GetPlatformVersion(node)
		imageProps, err := getImagePropsForEnvironment(platformVersion)
		if err != nil {
			return "", err
		}

		registry, hasRegistry := robot.Labels[internal.ROBOT_IMAGE_REGISTRY_LABEL_KEY]
		if !hasRegistry {
			return "", errors.New("registry is not found in label with key " + internal.ROBOT_IMAGE_REGISTRY_LABEL_KEY)
		}

		organization := imageProps.Organization
		repository := imageProps.Repository

		chosenElement := Element{}
		if robot.Spec.EnvironmentConfig.Domain == "plain" {
			for _, element := range imageProps.Domains["plain"] {
				if element.DevSpaceImage.UbuntuDistro != robot.Spec.EnvironmentConfig.DevSpaceImage.UbuntuDistro {
					continue
				}

				if element.DevSpaceImage.Desktop != robot.Spec.EnvironmentConfig.DevSpaceImage.Desktop {
					continue
				}

				if element.DevSpaceImage.Version != robot.Spec.EnvironmentConfig.DevSpaceImage.Version {
					continue
				}

				chosenElement = element
				break
			}

			if reflect.DeepEqual(chosenElement, Element{}) {
				return "", errors.New("environment is not supported")
			}
		} else {
			if domain, ok := imageProps.Domains[robot.Spec.EnvironmentConfig.Domain]; ok {
				for _, element := range domain {
					if element.Application.Name != robot.Spec.EnvironmentConfig.Application.Name {
						continue
					}

					if element.Application.Version != robot.Spec.EnvironmentConfig.Application.Version {
						continue
					}

					if element.DevSpaceImage.UbuntuDistro != robot.Spec.EnvironmentConfig.DevSpaceImage.UbuntuDistro {
						continue
					}

					if element.DevSpaceImage.Desktop != robot.Spec.EnvironmentConfig.DevSpaceImage.Desktop {
						continue
					}

					if element.DevSpaceImage.Version != robot.Spec.EnvironmentConfig.DevSpaceImage.Version {
						continue
					}

					chosenElement = element
					repository += "-" + robot.Spec.EnvironmentConfig.Domain
					tagBuilder.WriteString(chosenElement.Application.Name + "-")
					tagBuilder.WriteString(cosmodrome.FormatTag(chosenElement.Application.Version) + "-")
					break
				}

				if reflect.DeepEqual(chosenElement, Element{}) {
					return "", errors.New("environment is not supported")
				}

			} else {
				return "", errors.New("domain is not supported")
			}
		}

		tagBuilder.WriteString(chosenElement.DevSpaceImage.UbuntuDistro + "-")
		tagBuilder.WriteString(chosenElement.DevSpaceImage.Desktop + "-")
		tagBuilder.WriteString(chosenElement.DevSpaceImage.Version)

		imageBuilder.WriteString(filepath.Join(registry, organization, repository) + ":")
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

func getImagePropsForRobot(platformVersion, distro string) (Element, error) {

	resp, err := http.Get(internal.IMAGE_MAP_URL)
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
			for _, element := range v.Images.Domains["robotics"] {
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

func getImagePropsForEnvironment(platformVersion string) (Images, error) {

	resp, err := http.Get(internal.IMAGE_MAP_URL)
	if err != nil {
		return Images{}, err
	}

	defer resp.Body.Close()

	var yamlFile []byte
	if resp.StatusCode == http.StatusOK {
		yamlFile, err = io.ReadAll(resp.Body)
		if err != nil {
			return Images{}, err
		}
	}

	var platform Platform
	err = yaml.Unmarshal(yamlFile, &platform)
	if err != nil {
		return Images{}, err
	}

	var imageProps Images
	for _, v := range platform.Versions {
		if v.Version == platformVersion {
			imageProps = v.Images
		}
	}

	return imageProps, nil
}

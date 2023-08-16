package node

import (
	"errors"
	"io"
	"net/http"

	"github.com/robolaunch/robot-operator/internal"
	"gopkg.in/yaml.v2"
)

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

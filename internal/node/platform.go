package node

import (
	"context"
	"errors"
	"io"
	"net/http"

	"github.com/robolaunch/robot-operator/internal"
	"gopkg.in/yaml.v2"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/types"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

func getPlatform(ctx context.Context, r client.Client, platformVersion string) (Platform, error) {

	// try config map
	platformCmNamespacedName := types.NamespacedName{
		Name:      internal.IMAGE_MAP_CONFIG_MAP_NAME,
		Namespace: internal.IMAGE_MAP_CONFIG_MAP_NAMESPACE,
	}

	var yamlFile []byte

	platformCm := &corev1.ConfigMap{}
	cmErr := r.Get(ctx, platformCmNamespacedName, platformCm)
	if cmErr != nil {
		// try web
		resp, webErr := http.Get(internal.IMAGE_MAP_URL)
		if webErr != nil {
			return Platform{}, webErr
		}

		defer resp.Body.Close()

		if resp.StatusCode != http.StatusOK {
			return Platform{}, errors.New("cannot get platform.yaml from remote location " + internal.IMAGE_MAP_URL)
		}

		yamlFile, webErr = io.ReadAll(resp.Body)
		if webErr != nil {
			return Platform{}, webErr
		}
	} else {
		if yamlString, ok := platformCm.Data[internal.IMAGE_MAP_CONFIG_MAP_DATA_KEY]; ok {
			yamlFile = []byte(yamlString)
		} else {
			return Platform{}, errors.New("no data with key " + internal.IMAGE_MAP_CONFIG_MAP_DATA_KEY + " in config map " + platformCm.Name + "/" + platformCm.Namespace)
		}
	}

	var platform Platform
	err := yaml.Unmarshal(yamlFile, &platform)
	if err != nil {
		return Platform{}, err
	}

	return platform, nil

}

func getImagePropsForRobot(ctx context.Context, r client.Client, platformVersion, distro string) (Element, error) {

	platform, err := getPlatform(ctx, r, platformVersion)
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

func getImagePropsForEnvironment(ctx context.Context, r client.Client, platformVersion string) (Images, error) {

	platform, err := getPlatform(ctx, r, platformVersion)
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

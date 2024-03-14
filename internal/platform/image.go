package platform

import (
	"errors"
	"reflect"

	"github.com/robolaunch/platform/server/pkg/models"
)

func GetToolsImage(platformVersion string, application string, appVersion string) (string, error) {

	vpi, err := GetVersionedPlatformToolEnvironments(platformVersion)
	if err != nil {
		return "", err
	}

	selectedEnvironment := models.Environment{}

	if domain, ok := vpi.Domains["tools"]; ok {
		for _, environment := range domain.Environments {
			if environment.Application.Name == application && environment.Application.Version == appVersion {
				selectedEnvironment = environment
			}
		}
	} else {
		return "", errors.New("cannot find tools domain")
	}

	if reflect.DeepEqual(selectedEnvironment, models.Environment{}) {
		return "", errors.New("cannot find application '" + application + "' with version '" + appVersion + "'")
	}

	imageName := vpi.Repository + "/" + vpi.Organization + "/" + application + ":" + selectedEnvironment.Application.Version + "-" + selectedEnvironment.Base.Version

	return imageName, nil
}

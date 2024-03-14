package platform

import (
	"errors"
	"reflect"

	"github.com/robolaunch/platform/server/pkg/models"
	"github.com/robolaunch/robot-operator/internal/label"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

func GetToolsImage(obj metav1.Object, platformVersion string, application string, appVersion string) (string, error) {

	imageMeta := label.GetImageMeta(obj)

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

	imageName := imageMeta.Registry + "/" + vpi.Organization + "/" + application + ":" + selectedEnvironment.Application.Version + "-" + selectedEnvironment.Base.Version

	return imageName, nil
}

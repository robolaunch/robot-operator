package platform

import (
	"errors"
	"net/url"

	platformServer "github.com/robolaunch/platform/server/pkg"
	platformArtifact "github.com/robolaunch/platform/server/pkg/models"
	"github.com/robolaunch/robot-operator/internal"
)

func GetVersionedPlatformEnvironments(version string) (platformArtifact.VersionedPlatformImages, error) {

	environmentsResponse := platformServer.GetVersionedPlatformImagesResponse(
		map[string]string{
			"version": version,
		},
		url.Values{
			"url": []string{
				internal.DEFAULT_ENVIRONMENTS_URL,
			},
		},
	)

	if !environmentsResponse.Success {
		return platformArtifact.VersionedPlatformImages{}, errors.New(environmentsResponse.Message)
	}

	return platformArtifact.VersionedPlatformImages{}, nil
}

func GetVersionedPlatformToolEnvironments(version string) (platformArtifact.VersionedPlatformImages, error) {

	environmentsResponse := platformServer.GetVersionedPlatformImagesResponse(
		map[string]string{
			"version": version,
		},
		url.Values{
			"url": []string{
				internal.DEFAULT_TOOLS_ENVIRONMENTS_URL,
			},
		},
	)

	if !environmentsResponse.Success {
		return platformArtifact.VersionedPlatformImages{}, errors.New(environmentsResponse.Message)
	}

	return *environmentsResponse.Response, nil
}

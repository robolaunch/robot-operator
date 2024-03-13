package platform

import (
	"errors"
	"net/url"

	platformServer "github.com/robolaunch/platform/server/pkg"
	platformArtifact "github.com/robolaunch/platform/server/pkg/models"
	"github.com/robolaunch/robot-operator/internal"
)

func GetPlatform() (platformArtifact.Robolaunch, error) {

	platformResponse := platformServer.GetPlatformResponse(
		url.Values{
			"url": []string{
				internal.DEFAULT_PLATFORM_URL,
			},
		},
	)

	if !platformResponse.Success {
		return platformArtifact.Robolaunch{}, errors.New(platformResponse.Message)
	}

	return *platformResponse.Response, nil
}

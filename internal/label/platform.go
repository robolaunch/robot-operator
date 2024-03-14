package label

import (
	"github.com/robolaunch/robot-operator/internal"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

type PlatformMeta struct {
	Version string
	Domain  string
}

func GetPlatformMeta(obj metav1.Object) *PlatformMeta {
	platformMeta := &PlatformMeta{}
	labels := obj.GetLabels()

	if version, ok := labels[internal.PLATFORM_VERSION_LABEL_KEY]; ok {
		platformMeta.Version = version
	}

	if domain, ok := labels[internal.DOMAIN_LABEL_KEY]; ok {
		platformMeta.Domain = domain
	}

	return platformMeta
}

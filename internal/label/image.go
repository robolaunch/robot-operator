package label

import (
	"github.com/robolaunch/robot-operator/internal"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

type ImageMeta struct {
	Registry   string
	User       string
	Repository string
	Tag        string
}

func GetImageMeta(obj metav1.Object) *ImageMeta {

	imageMeta := &ImageMeta{}
	labels := obj.GetLabels()

	if registry, ok := labels[internal.IMAGE_REGISTRY_LABEL_KEY]; ok {
		imageMeta.Registry = registry
	}

	if user, ok := labels[internal.IMAGE_USER_LABEL_KEY]; ok {
		imageMeta.User = user
	}

	if repository, ok := labels[internal.IMAGE_REPOSITORY_LABEL_KEY]; ok {
		imageMeta.Repository = repository
	}

	if tag, ok := labels[internal.IMAGE_TAG_LABEL_KEY]; ok {
		imageMeta.Tag = tag
	}

	return imageMeta
}

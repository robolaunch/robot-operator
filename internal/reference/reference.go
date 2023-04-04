package reference

import (
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

func SetReference(ref *corev1.ObjectReference, typeMeta metav1.TypeMeta, objectMeta metav1.ObjectMeta) {
	ref.APIVersion = typeMeta.APIVersion
	ref.Kind = typeMeta.Kind
	ref.Name = objectMeta.GetName()
	ref.Namespace = objectMeta.GetNamespace()
	ref.ResourceVersion = objectMeta.GetResourceVersion()
	ref.UID = objectMeta.GetUID()
}

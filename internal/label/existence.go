package label

import (
	robotErr "github.com/robolaunch/robot-operator/internal/error"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

func CheckLabelExistence(objMeta metav1.ObjectMeta, typeMeta metav1.TypeMeta, labelKeys []string) error {
	labels := objMeta.GetLabels()
	for _, key := range labelKeys {
		if _, ok := labels[key]; !ok {
			return &robotErr.LabelNotFoundError{
				LabelKey:          key,
				ResourceKind:      typeMeta.Kind,
				ResourceName:      objMeta.Name,
				ResourceNamespace: objMeta.Namespace,
			}
		}
	}
	return nil
}

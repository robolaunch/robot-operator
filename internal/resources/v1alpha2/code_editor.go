package v1alpha2_resources

import (
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"

	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

func GetCodeEditorPersistentVolumeClaim(codeEditor *robotv1alpha2.CodeEditor, pvcNamespacedName *types.NamespacedName, key int) *corev1.PersistentVolumeClaim {

	pvc := corev1.PersistentVolumeClaim{
		ObjectMeta: metav1.ObjectMeta{
			Name:      pvcNamespacedName.Name,
			Namespace: pvcNamespacedName.Namespace,
			Labels:    codeEditor.Labels,
		},
		Spec: codeEditor.Spec.VolumeClaimTemplates[key].Spec,
	}

	return &pvc

}

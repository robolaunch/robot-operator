package spawn

import (
	"strconv"
	"strings"

	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/resource"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
)

func GetPersistentVolumeClaim(
	robot *robotv1alpha1.Robot,
	pvcNamespacedName *types.NamespacedName,
) *corev1.PersistentVolumeClaim {

	pvc := corev1.PersistentVolumeClaim{
		ObjectMeta: metav1.ObjectMeta{
			Name:      pvcNamespacedName.Name,
			Namespace: pvcNamespacedName.Namespace,
		},
		Spec: corev1.PersistentVolumeClaimSpec{
			StorageClassName: &robot.Spec.Storage.StorageClassConfig.Name,
			AccessModes: []corev1.PersistentVolumeAccessMode{
				robot.Spec.Storage.StorageClassConfig.AccessMode,
			},
			Resources: corev1.ResourceRequirements{
				Limits: corev1.ResourceList{
					corev1.ResourceName(corev1.ResourceStorage): resource.MustParse(getClaimStorage(pvcNamespacedName, robot.Spec.Storage.Amount)),
				},
				Requests: corev1.ResourceList{
					corev1.ResourceName(corev1.ResourceStorage): resource.MustParse(getClaimStorage(pvcNamespacedName, robot.Spec.Storage.Amount)),
				},
			},
		},
	}

	return &pvc
}

func getClaimStorage(pvc *types.NamespacedName, totalStorage int) string {
	storageInt := 0

	if strings.Contains(pvc.Name, "pvc-var") {
		storageInt = totalStorage / 20
	} else if strings.Contains(pvc.Name, "pvc-opt") {
		storageInt = 3 * totalStorage / 10
	} else if strings.Contains(pvc.Name, "pvc-usr") {
		storageInt = totalStorage * 5 / 10
	} else if strings.Contains(pvc.Name, "pvc-etc") {
		storageInt = totalStorage / 20
	} else if strings.Contains(pvc.Name, "pvc-display") {
		storageInt = 100
	} else if strings.Contains(pvc.Name, "pvc-workspace") {
		storageInt = totalStorage / 10
	} else {
		storageInt = 0
	}
	return strconv.Itoa(storageInt) + "M"

}

package v1alpha2_resources

import (
	appsv1 "k8s.io/api/apps/v1"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

func GetDiscoveryServer(ros2Workload *robotv1alpha2.ROS2Workload, dsNamespacedName *types.NamespacedName) *robotv1alpha1.DiscoveryServer {

	discoveryServer := robotv1alpha1.DiscoveryServer{
		ObjectMeta: metav1.ObjectMeta{
			Name:      dsNamespacedName.Name,
			Namespace: dsNamespacedName.Namespace,
			Labels:    ros2Workload.Labels,
		},
		Spec: ros2Workload.Spec.DiscoveryServerTemplate,
	}

	return &discoveryServer

}

func GetROS2Bridge(ros2Workload *robotv1alpha2.ROS2Workload, r2bNamespacedName *types.NamespacedName) *robotv1alpha2.ROS2Bridge {

	ros2Bridge := robotv1alpha2.ROS2Bridge{
		ObjectMeta: metav1.ObjectMeta{
			Name:      r2bNamespacedName.Name,
			Namespace: r2bNamespacedName.Namespace,
			Labels:    ros2Workload.Labels,
		},
		Spec: ros2Workload.Spec.ROS2BridgeTemplate,
	}

	return &ros2Bridge

}

func GetPersistentVolumeClaim(ros2Workload *robotv1alpha2.ROS2Workload, pvcNamespacedName *types.NamespacedName, key int) *corev1.PersistentVolumeClaim {

	pvc := corev1.PersistentVolumeClaim{
		ObjectMeta: metav1.ObjectMeta{
			Name:      pvcNamespacedName.Name,
			Namespace: pvcNamespacedName.Namespace,
			Labels:    ros2Workload.Labels,
		},
		Spec: ros2Workload.Spec.VolumeClaimTemplates[key].Spec,
	}

	return &pvc

}

func GetStatefulSet(ros2Workload *robotv1alpha2.ROS2Workload, ssNamespacedName *types.NamespacedName, key int) *appsv1.StatefulSet {

	container := ros2Workload.Spec.Containers[key]
	ssAliasLabels := ros2Workload.Labels
	ssAliasLabels["robolaunch.io/alias"] = container.Container.Name

	statefulSet := appsv1.StatefulSet{
		ObjectMeta: metav1.ObjectMeta{
			Name:      ssNamespacedName.Name,
			Namespace: ssNamespacedName.Namespace,
			Labels:    ssAliasLabels,
		},
		Spec: appsv1.StatefulSetSpec{
			Replicas: ros2Workload.Spec.Containers[key].Replicas,
			Template: corev1.PodTemplateSpec{
				Spec: corev1.PodSpec{
					Containers: []corev1.Container{
						container.Container,
					},
				},
			},
		},
	}

	return &statefulSet

}

package v1alpha2_resources

import (
	appsv1 "k8s.io/api/apps/v1"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"

	"github.com/robolaunch/robot-operator/internal"
	configure "github.com/robolaunch/robot-operator/internal/configure/v1alpha2"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

func GetROS2WorkloadDiscoveryServer(ros2Workload *robotv1alpha2.ROS2Workload, dsNamespacedName *types.NamespacedName) *robotv1alpha1.DiscoveryServer {

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

func GetROS2WorkloadROS2Bridge(ros2Workload *robotv1alpha2.ROS2Workload, r2bNamespacedName *types.NamespacedName) *robotv1alpha2.ROS2Bridge {

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

func GetROS2WorkloadPersistentVolumeClaim(ros2Workload *robotv1alpha2.ROS2Workload, pvcNamespacedName *types.NamespacedName, key int) *corev1.PersistentVolumeClaim {

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

func GetROS2WorkloadStatefulSet(ros2Workload *robotv1alpha2.ROS2Workload, ssNamespacedName *types.NamespacedName, key int, node corev1.Node) *appsv1.StatefulSet {

	cfg := configure.PodSpecConfigInjector{}

	container := ros2Workload.Spec.LaunchContainers[key]
	ssAliasLabels := ros2Workload.Labels
	ssAliasLabels["robolaunch.io/alias"] = container.Container.Name

	podSpec := corev1.PodSpec{
		Containers: []corev1.Container{
			container.Container,
		},
	}

	cfg.InjectDiscoveryServerConnection(&podSpec, ros2Workload.Status.DiscoveryServerStatus.Status.ConnectionInfo)
	cfg.InjectROSDomainID(&podSpec, ros2Workload.Spec.DiscoveryServerTemplate.DomainID)
	cfg.InjectRMWImplementationConfiguration(&podSpec, "rmw_fastrtps_cpp")
	cfg.InjectImagePullPolicy(&podSpec)
	cfg.SchedulePod(&podSpec, ros2Workload)
	cfg.InjectTimezone(&podSpec, node)
	cfg.InjectRuntimeClass(&podSpec, *ros2Workload, node)
	cfg.InjectVolumeConfiguration(&podSpec, *ros2Workload)

	statefulSet := appsv1.StatefulSet{
		ObjectMeta: metav1.ObjectMeta{
			Name:      ssNamespacedName.Name,
			Namespace: ssNamespacedName.Namespace,
			Labels:    ssAliasLabels,
		},
		Spec: appsv1.StatefulSetSpec{
			Replicas: ros2Workload.Spec.LaunchContainers[key].Replicas,
			Selector: &metav1.LabelSelector{
				MatchLabels: map[string]string{
					internal.ROS2_WORKLOAD_CONTAINER_SELECTOR_LABEL_KEY: container.Container.Name,
				},
			},
			Template: corev1.PodTemplateSpec{
				ObjectMeta: metav1.ObjectMeta{
					Labels: map[string]string{
						internal.ROS2_WORKLOAD_CONTAINER_SELECTOR_LABEL_KEY: container.Container.Name,
					},
				},
				Spec: podSpec,
			},
		},
	}

	return &statefulSet

}

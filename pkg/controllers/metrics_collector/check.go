package metrics_collector

import (
	"context"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/apimachinery/pkg/selection"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

func (r *MetricsCollectorReconciler) reconcileGetPods(ctx context.Context, instance *robotv1alpha1.MetricsCollector) error {

	// Get attached build objects for this robot
	requirements := []labels.Requirement{}
	newReq, err := labels.NewRequirement(internal.TARGET_ROBOT_LABEL_KEY, selection.In, []string{label.GetTargetRobot(instance)})
	if err != nil {
		return err
	}
	requirements = append(requirements, *newReq)

	robotSelector := labels.NewSelector().Add(requirements...)

	podList := &corev1.PodList{}
	err = r.List(context.TODO(), podList, &client.ListOptions{Namespace: instance.Namespace, LabelSelector: robotSelector})
	if err != nil {
		return err
	}

	instance.Status.ComponentMetrics = []robotv1alpha1.ComponentMetricStatus{}

	for _, pod := range podList.Items {
		for _, container := range pod.Spec.Containers {
			instance.Status.ComponentMetrics = append(instance.Status.ComponentMetrics, robotv1alpha1.ComponentMetricStatus{
				OwnerResourceReference: pod.OwnerReferences[0],
				PodReference: corev1.ObjectReference{
					Kind:            pod.Kind,
					Namespace:       pod.Namespace,
					Name:            pod.Name,
					UID:             pod.UID,
					APIVersion:      pod.APIVersion,
					ResourceVersion: pod.ResourceVersion,
				},
				ContainerName: container.Name,
			})
		}
	}

	return nil
}

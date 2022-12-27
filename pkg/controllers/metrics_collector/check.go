package metrics_collector

import (
	"context"
	"sort"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/apimachinery/pkg/selection"
	"k8s.io/apimachinery/pkg/types"
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

	newComponentMetrics := []robotv1alpha1.ComponentMetricStatus{}

	for _, pod := range podList.Items {
		for _, container := range pod.Spec.Containers {
			newComponentMetrics = append(newComponentMetrics, robotv1alpha1.ComponentMetricStatus{
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

	for k1, m := range newComponentMetrics {
		for _, oldM := range instance.Status.ComponentMetrics {
			if oldM.PodReference.Name == m.PodReference.Name && oldM.ContainerName == m.ContainerName {
				// save cpu utilization data
				m.CPUUtilization = oldM.CPUUtilization
				// save network load utilization data
				m.NetworkLoadUtilization = oldM.NetworkLoadUtilization
				newComponentMetrics[k1] = m
			}
		}
	}

	sort.SliceStable(newComponentMetrics, func(i, j int) bool {
		if newComponentMetrics[i].PodReference.Name < newComponentMetrics[j].PodReference.Name {
			return true
		}

		if newComponentMetrics[i].PodReference.Name == newComponentMetrics[j].PodReference.Name {
			if newComponentMetrics[i].ContainerName < newComponentMetrics[j].ContainerName {
				return true
			}
		}

		return false
	})

	instance.Status.ComponentMetrics = newComponentMetrics

	return nil
}

func (r *MetricsCollectorReconciler) reconcileCheckNode(ctx context.Context, instance *robotv1alpha1.MetricsCollector) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	node := &corev1.Node{}
	err = r.Get(ctx, types.NamespacedName{Name: robot.Status.NodeName}, node)
	if err != nil {
		return err
	}

	instance.Status.Allocatable = node.Status.Allocatable

	return nil
}

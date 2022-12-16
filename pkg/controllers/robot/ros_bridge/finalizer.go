package ros_bridge

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/apimachinery/pkg/watch"
	"sigs.k8s.io/controller-runtime/pkg/controller/controllerutil"
)

func (r *ROSBridgeReconciler) reconcileCheckDeletion(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

	rosBridgeFinalizer := "rosbridge.roboscale.io/finalizer"

	if instance.DeletionTimestamp.IsZero() {

		if !controllerutil.ContainsFinalizer(instance, rosBridgeFinalizer) {
			controllerutil.AddFinalizer(instance, rosBridgeFinalizer)
			if err := r.Update(ctx, instance); err != nil {
				return err
			}
		}

	} else {

		if controllerutil.ContainsFinalizer(instance, rosBridgeFinalizer) {

			err := r.waitForPodDeletion(ctx, instance)
			if err != nil {
				return err
			}

			err = r.waitForServiceDeletion(ctx, instance)
			if err != nil {
				return err
			}

			controllerutil.RemoveFinalizer(instance, rosBridgeFinalizer)
			if err := r.Update(ctx, instance); err != nil {
				return err
			}
		}

		return errors.NewNotFound(schema.GroupResource{
			Group:    instance.GetObjectKind().GroupVersionKind().Group,
			Resource: instance.GetObjectKind().GroupVersionKind().Kind,
		}, instance.Name)
	}

	return nil
}

func (r *ROSBridgeReconciler) waitForPodDeletion(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

	rosBridgePodQuery := &corev1.Pod{}
	err := r.Get(ctx, *instance.GetBridgePodMetadata(), rosBridgePodQuery)
	if err != nil && errors.IsNotFound(err) {
		return nil
	} else if err != nil {
		return err
	} else {
		logger.Info("FINALIZER: ROS bridge pod is being deleted.")
		err := r.Delete(ctx, rosBridgePodQuery)
		if err != nil {
			return err
		}

		instance.Status.Phase = robotv1alpha1.BridgePhaseDeletingPod
		err = r.reconcileUpdateInstanceStatus(ctx, instance)
		if err != nil {
			return err
		}

		resourceInterface := r.DynamicClient.Resource(schema.GroupVersionResource{
			Group:    rosBridgePodQuery.GroupVersionKind().Group,
			Version:  rosBridgePodQuery.GroupVersionKind().Version,
			Resource: "pods",
		})
		podWatcher, err := resourceInterface.Watch(ctx, metav1.ListOptions{
			FieldSelector: "metadata.name=" + instance.GetBridgePodMetadata().Name,
		})
		if err != nil {
			return err
		}

		defer podWatcher.Stop()

		podDeleted := false
		for {
			if !podDeleted {
				select {
				case event := <-podWatcher.ResultChan():

					if event.Type == watch.Deleted {
						logger.Info("FINALIZER: ROS bridge pod is deleted gracefully.")
						podDeleted = true
					}
				}
			} else {
				break
			}

		}
	}
	return nil
}

func (r *ROSBridgeReconciler) waitForServiceDeletion(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

	rosBridgeServiceQuery := &corev1.Service{}
	err := r.Get(ctx, *instance.GetBridgeServiceMetadata(), rosBridgeServiceQuery)
	if err != nil && errors.IsNotFound(err) {
		return nil
	} else if err != nil {
		return err
	} else {
		logger.Info("FINALIZER: ROS bridge service is being deleted.")
		err := r.Delete(ctx, rosBridgeServiceQuery)
		if err != nil {
			return err
		}

		instance.Status.Phase = robotv1alpha1.BridgePhaseDeletingService
		err = r.reconcileUpdateInstanceStatus(ctx, instance)
		if err != nil {
			return err
		}

		resourceInterface := r.DynamicClient.Resource(schema.GroupVersionResource{
			Group:    rosBridgeServiceQuery.GroupVersionKind().Group,
			Version:  rosBridgeServiceQuery.GroupVersionKind().Version,
			Resource: "services",
		})
		svcWatcher, err := resourceInterface.Watch(ctx, metav1.ListOptions{
			FieldSelector: "metadata.name=" + instance.GetBridgeServiceMetadata().Name,
		})
		if err != nil {
			return err
		}

		defer svcWatcher.Stop()

		svcDeleted := false
		for {
			if !svcDeleted {
				select {
				case event := <-svcWatcher.ResultChan():

					if event.Type == watch.Deleted {
						logger.Info("FINALIZER: ROS bridge service is deleted gracefully.")
						svcDeleted = true
					}
				}
			} else {
				break
			}

		}
	}
	return nil
}

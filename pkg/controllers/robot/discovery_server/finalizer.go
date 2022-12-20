package discovery_server

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/apimachinery/pkg/watch"
	"sigs.k8s.io/controller-runtime/pkg/controller/controllerutil"
)

func (r *DiscoveryServerReconciler) reconcileCheckDeletion(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	discoveryServerFinalizer := "discoveryserver.roboscale.io/finalizer"

	if instance.DeletionTimestamp.IsZero() {

		if !controllerutil.ContainsFinalizer(instance, discoveryServerFinalizer) {
			controllerutil.AddFinalizer(instance, discoveryServerFinalizer)
			if err := r.Update(ctx, instance); err != nil {
				return err
			}
		}

	} else {

		if controllerutil.ContainsFinalizer(instance, discoveryServerFinalizer) {

			// err := r.waitForConfigMapDeletion(ctx, instance)
			// if err != nil {
			// 	return err
			// }

			err := r.waitForPodDeletion(ctx, instance)
			if err != nil {
				return err
			}

			// err = r.waitForServiceDeletion(ctx, instance)
			// if err != nil {
			// 	return err
			// }

			controllerutil.RemoveFinalizer(instance, discoveryServerFinalizer)
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

func (r *DiscoveryServerReconciler) waitForConfigMapDeletion(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	instance.Status.Phase = robotv1alpha1.DiscoveryServerPhaseDeletingConfigMap
	err := r.reconcileUpdateInstanceStatus(ctx, instance)
	if err != nil {
		return err
	}

	discoveryServerConfigMapQuery := &corev1.ConfigMap{}
	err = r.Get(ctx, *instance.GetDiscoveryServerConfigMapMetadata(), discoveryServerConfigMapQuery)
	if err != nil && errors.IsNotFound(err) {
		return nil
	} else if err != nil {
		return err
	} else {
		logger.Info("FINALIZER: Discovery server config map is being deleted.")
		err := r.Delete(ctx, discoveryServerConfigMapQuery)
		if err != nil {
			return err
		}
	}

	discoveryServerConfigMapQuery = &corev1.ConfigMap{}
	err = r.Get(ctx, *instance.GetDiscoveryServerConfigMapMetadata(), discoveryServerConfigMapQuery)
	if err != nil && errors.IsNotFound(err) {
		return nil
	} else if err != nil {
		return err
	} else {

		resourceInterface := r.DynamicClient.Resource(schema.GroupVersionResource{
			Group:    discoveryServerConfigMapQuery.GroupVersionKind().Group,
			Version:  discoveryServerConfigMapQuery.GroupVersionKind().Version,
			Resource: "configmaps",
		})
		cmWatcher, err := resourceInterface.Watch(ctx, metav1.ListOptions{
			FieldSelector: "metadata.name=" + instance.GetDiscoveryServerConfigMapMetadata().Name,
		})
		if err != nil {
			return err
		}

		defer cmWatcher.Stop()

		cmDeleted := false
		for {
			if !cmDeleted {
				select {
				case event := <-cmWatcher.ResultChan():

					if event.Type == watch.Deleted {
						logger.Info("FINALIZER: Discovery server config map is deleted gracefully.")
						cmDeleted = true
					}
				}
			} else {
				break
			}

		}
	}
	return nil
}

func (r *DiscoveryServerReconciler) waitForPodDeletion(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	discoveryServerPodQuery := &corev1.Pod{}
	err := r.Get(ctx, *instance.GetDiscoveryServerPodMetadata(), discoveryServerPodQuery)
	if err != nil && errors.IsNotFound(err) {
		return nil
	} else if err != nil {
		return err
	} else {
		logger.Info("FINALIZER: Discovery server pod is being deleted.")
		err := r.Delete(ctx, discoveryServerPodQuery)
		if err != nil {
			return err
		}

		instance.Status.Phase = robotv1alpha1.DiscoveryServerPhaseDeletingPod
		err = r.reconcileUpdateInstanceStatus(ctx, instance)
		if err != nil {
			return err
		}

		resourceInterface := r.DynamicClient.Resource(schema.GroupVersionResource{
			Group:    discoveryServerPodQuery.GroupVersionKind().Group,
			Version:  discoveryServerPodQuery.GroupVersionKind().Version,
			Resource: "pods",
		})
		podWatcher, err := resourceInterface.Watch(ctx, metav1.ListOptions{
			FieldSelector: "metadata.name=" + instance.GetDiscoveryServerPodMetadata().Name,
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
						logger.Info("FINALIZER: Discovery server pod is deleted gracefully.")
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

func (r *DiscoveryServerReconciler) waitForServiceDeletion(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	instance.Status.Phase = robotv1alpha1.DiscoveryServerPhaseDeletingService
	err := r.reconcileUpdateInstanceStatus(ctx, instance)
	if err != nil {
		return err
	}

	discoveryServerServiceQuery := &corev1.Service{}
	err = r.Get(ctx, *instance.GetDiscoveryServerServiceMetadata(), discoveryServerServiceQuery)
	if err != nil && errors.IsNotFound(err) {
		return nil
	} else if err != nil {
		return err
	} else {
		logger.Info("FINALIZER: Discovery server service is being deleted.")
		err := r.Delete(ctx, discoveryServerServiceQuery)
		if err != nil {
			return err
		}
	}

	discoveryServerServiceQuery = &corev1.Service{}
	err = r.Get(ctx, *instance.GetDiscoveryServerServiceMetadata(), discoveryServerServiceQuery)
	if err != nil && errors.IsNotFound(err) {
		return nil
	} else if err != nil {
		return err
	} else {

		resourceInterface := r.DynamicClient.Resource(schema.GroupVersionResource{
			Group:    discoveryServerServiceQuery.GroupVersionKind().Group,
			Version:  discoveryServerServiceQuery.GroupVersionKind().Version,
			Resource: "services",
		})
		svcWatcher, err := resourceInterface.Watch(ctx, metav1.ListOptions{
			FieldSelector: "metadata.name=" + instance.GetDiscoveryServerServiceMetadata().Name,
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
						logger.Info("FINALIZER: Discovery server service is deleted gracefully.")
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

package robot

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/apimachinery/pkg/watch"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/controller/controllerutil"
)

func (r *RobotReconciler) reconcileCheckDeletion(ctx context.Context, instance *robotv1alpha1.Robot) error {

	robotFinalizer := "robot.roboscale.io/finalizer"

	if instance.DeletionTimestamp.IsZero() {

		if !controllerutil.ContainsFinalizer(instance, robotFinalizer) {
			controllerutil.AddFinalizer(instance, robotFinalizer)
			if err := r.Update(ctx, instance); err != nil {
				return err
			}
		}

	} else {

		if controllerutil.ContainsFinalizer(instance, robotFinalizer) {

			err := r.waitForROSBridgeDeletion(ctx, instance)
			if err != nil {
				return err
			}

			err = r.waitForDiscoveryServerDeletion(ctx, instance)
			if err != nil {
				return err
			}

			err = r.waitForLoaderJobDeletion(ctx, instance)
			if err != nil {
				return err
			}

			err = r.waitForPersistentVolumeClaimDeletion(ctx, instance, instance.GetPVCVarMetadata())
			if err != nil {
				return err
			}

			err = r.waitForPersistentVolumeClaimDeletion(ctx, instance, instance.GetPVCEtcMetadata())
			if err != nil {
				return err
			}

			err = r.waitForPersistentVolumeClaimDeletion(ctx, instance, instance.GetPVCOptMetadata())
			if err != nil {
				return err
			}

			err = r.waitForPersistentVolumeClaimDeletion(ctx, instance, instance.GetPVCUsrMetadata())
			if err != nil {
				return err
			}

			err = r.waitForPersistentVolumeClaimDeletion(ctx, instance, instance.GetPVCDisplayMetadata())
			if err != nil {
				return err
			}

			err = r.waitForPersistentVolumeClaimDeletion(ctx, instance, instance.GetPVCWorkspaceMetadata())
			if err != nil {
				return err
			}

			controllerutil.RemoveFinalizer(instance, robotFinalizer)
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

func (r *RobotReconciler) waitForROSBridgeDeletion(ctx context.Context, instance *robotv1alpha1.Robot) error {

	rosBridgeQuery := &robotv1alpha1.ROSBridge{}
	err := r.Get(ctx, *instance.GetROSBridgeMetadata(), rosBridgeQuery)
	if err != nil && errors.IsNotFound(err) {
		return nil
	} else if err != nil {
		return err
	} else {
		logger.Info("FINALIZER: ROS bridge is being deleted.")
		err := r.Delete(ctx, rosBridgeQuery)
		if err != nil {
			return err
		}

		instance.Status.Phase = robotv1alpha1.RobotPhaseDeletingBridge
		err = r.reconcileUpdateInstanceStatus(ctx, instance)
		if err != nil {
			return err
		}

		resourceInterface := r.DynamicClient.Resource(schema.GroupVersionResource{
			Group:    rosBridgeQuery.GroupVersionKind().Group,
			Version:  rosBridgeQuery.GroupVersionKind().Version,
			Resource: "rosbridges",
		})
		bridgeWatcher, err := resourceInterface.Watch(ctx, metav1.ListOptions{
			FieldSelector: "metadata.name=" + instance.GetROSBridgeMetadata().Name,
		})
		if err != nil {
			return err
		}

		defer bridgeWatcher.Stop()

		bridgeDeleted := false
		for {
			if !bridgeDeleted {
				select {
				case event := <-bridgeWatcher.ResultChan():

					if event.Type == watch.Deleted {
						logger.Info("FINALIZER: ROS bridge is deleted gracefully.")
						bridgeDeleted = true
					}
				}
			} else {
				break
			}

		}
	}
	return nil
}

func (r *RobotReconciler) waitForDiscoveryServerDeletion(ctx context.Context, instance *robotv1alpha1.Robot) error {

	discoveryServerQuery := &robotv1alpha1.DiscoveryServer{}
	err := r.Get(ctx, *instance.GetDiscoveryServerMetadata(), discoveryServerQuery)
	if err != nil && errors.IsNotFound(err) {
		return nil
	} else if err != nil {
		return err
	} else {
		logger.Info("FINALIZER: Discovery server is being deleted.")
		err := r.Delete(ctx, discoveryServerQuery)
		if err != nil {
			return err
		}

		instance.Status.Phase = robotv1alpha1.RobotPhaseDeletingDiscoveryServer
		err = r.reconcileUpdateInstanceStatus(ctx, instance)
		if err != nil {
			return err
		}

		resourceInterface := r.DynamicClient.Resource(schema.GroupVersionResource{
			Group:    discoveryServerQuery.GroupVersionKind().Group,
			Version:  discoveryServerQuery.GroupVersionKind().Version,
			Resource: "discoveryservers",
		})
		dsWatcher, err := resourceInterface.Watch(ctx, metav1.ListOptions{
			FieldSelector: "metadata.name=" + instance.GetDiscoveryServerMetadata().Name,
		})
		if err != nil {
			return err
		}

		defer dsWatcher.Stop()

		dsDeleted := false
		for {
			if !dsDeleted {
				select {
				case event := <-dsWatcher.ResultChan():

					if event.Type == watch.Deleted {
						logger.Info("FINALIZER: Discovery server is deleted gracefully.")
						dsDeleted = true
					}
				}
			} else {
				break
			}

		}
	}
	return nil
}

func (r *RobotReconciler) waitForLoaderJobDeletion(ctx context.Context, instance *robotv1alpha1.Robot) error {

	instance.Status.Phase = robotv1alpha1.RobotPhaseDeletingLoaderJob
	err := r.reconcileUpdateInstanceStatus(ctx, instance)
	if err != nil {
		return err
	}

	loaderJobQuery := &batchv1.Job{}
	err = r.Get(ctx, *instance.GetLoaderJobMetadata(), loaderJobQuery)
	if err != nil && errors.IsNotFound(err) {
		return nil
	} else if err != nil {
		return err
	} else {
		logger.Info("FINALIZER: Loader job is being deleted.")
		propagationPolicy := metav1.DeletePropagationBackground
		err := r.Delete(ctx, loaderJobQuery, &client.DeleteOptions{
			PropagationPolicy: &propagationPolicy,
		})
		if err != nil {
			return err
		}
	}

	loaderJobQuery = &batchv1.Job{}
	err = r.Get(ctx, *instance.GetLoaderJobMetadata(), loaderJobQuery)
	if err != nil && errors.IsNotFound(err) {
		return nil
	} else if err != nil {
		return err
	} else {

		resourceInterface := r.DynamicClient.Resource(schema.GroupVersionResource{
			Group:    loaderJobQuery.GroupVersionKind().Group,
			Version:  loaderJobQuery.GroupVersionKind().Version,
			Resource: "jobs",
		})
		jobWatcher, err := resourceInterface.Watch(ctx, metav1.ListOptions{
			FieldSelector: "metadata.name=" + instance.GetLoaderJobMetadata().Name,
		})
		if err != nil {
			return err
		}

		defer jobWatcher.Stop()

		jobDeleted := false
		for {
			if !jobDeleted {
				select {
				case event := <-jobWatcher.ResultChan():

					if event.Type == watch.Deleted {
						logger.Info("FINALIZER: Loader job is deleted gracefully.")
						jobDeleted = true
					}
				}
			} else {
				break
			}

		}
	}
	return nil
}

func (r *RobotReconciler) waitForPersistentVolumeClaimDeletion(ctx context.Context, instance *robotv1alpha1.Robot, pvcNamespacedName *types.NamespacedName) error {

	pvcQuery := &corev1.PersistentVolumeClaim{}
	err := r.Get(ctx, *pvcNamespacedName, pvcQuery)
	if err != nil && errors.IsNotFound(err) {
		return nil
	} else if err != nil {
		return err
	} else {
		logger.Info("FINALIZER: PVC " + pvcNamespacedName.Name + " is being deleted.")
		err := r.Delete(ctx, pvcQuery)
		if err != nil {
			return err
		}

		instance.Status.Phase = robotv1alpha1.RobotPhaseDeletingVolumes
		err = r.reconcileUpdateInstanceStatus(ctx, instance)
		if err != nil {
			return err
		}

		resourceInterface := r.DynamicClient.Resource(schema.GroupVersionResource{
			Group:    pvcQuery.GroupVersionKind().Group,
			Version:  pvcQuery.GroupVersionKind().Version,
			Resource: "persistentvolumeclaims",
		})
		pvcWatcher, err := resourceInterface.Watch(ctx, metav1.ListOptions{
			FieldSelector: "metadata.name=" + pvcNamespacedName.Name,
		})
		if err != nil {
			return err
		}

		defer pvcWatcher.Stop()

		pvcDeleted := false
		for {
			if !pvcDeleted {
				select {
				case event := <-pvcWatcher.ResultChan():

					if event.Type == watch.Deleted {
						logger.Info("FINALIZER: PVC " + pvcNamespacedName.Name + " is deleted gracefully.")
						pvcDeleted = true
					}
				}
			} else {
				break
			}

		}
	}
	return nil
}

// func (r *ROSBridgeReconciler) waitForPodDeletion(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

// 	rosBridgePodQuery := &corev1.Pod{}
// 	err := r.Get(ctx, *instance.GetBridgePodMetadata(), rosBridgePodQuery)
// 	if err != nil && errors.IsNotFound(err) {
// 		return nil
// 	} else if err != nil {
// 		return err
// 	} else {
// 		logger.Info("FINALIZER: ROS bridge pod is being deleted.")
// 		err := r.Delete(ctx, rosBridgePodQuery)
// 		if err != nil {
// 			return err
// 		}

// 		instance.Status.Phase = robotv1alpha1.BridgePhaseDeletingPod
// 		err = r.reconcileUpdateInstanceStatus(ctx, instance)
// 		if err != nil {
// 			return err
// 		}

// 		resourceInterface := r.DynamicClient.Resource(schema.GroupVersionResource{
// 			Group:    rosBridgePodQuery.GroupVersionKind().Group,
// 			Version:  rosBridgePodQuery.GroupVersionKind().Version,
// 			Resource: "pods",
// 		})
// 		podWatcher, err := resourceInterface.Watch(ctx, metav1.ListOptions{
// 			FieldSelector: "metadata.name=" + instance.GetBridgePodMetadata().Name,
// 		})
// 		if err != nil {
// 			return err
// 		}

// 		defer podWatcher.Stop()

// 		podDeleted := false
// 		for {
// 			if !podDeleted {
// 				select {
// 				case event := <-podWatcher.ResultChan():

// 					if event.Type == watch.Deleted {
// 						logger.Info("FINALIZER: ROS bridge pod is deleted gracefully.")
// 						podDeleted = true
// 					}
// 				}
// 			} else {
// 				break
// 			}

// 		}
// 	}
// 	return nil
// }

// func (r *ROSBridgeReconciler) waitForServiceDeletion(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

// 	rosBridgeServiceQuery := &corev1.Service{}
// 	err := r.Get(ctx, *instance.GetBridgeServiceMetadata(), rosBridgeServiceQuery)
// 	if err != nil && errors.IsNotFound(err) {
// 		return nil
// 	} else if err != nil {
// 		return err
// 	} else {
// 		logger.Info("FINALIZER: ROS bridge service is being deleted.")
// 		err := r.Delete(ctx, rosBridgeServiceQuery)
// 		if err != nil {
// 			return err
// 		}

// 		instance.Status.Phase = robotv1alpha1.BridgePhaseDeletingService
// 		err = r.reconcileUpdateInstanceStatus(ctx, instance)
// 		if err != nil {
// 			return err
// 		}

// 		resourceInterface := r.DynamicClient.Resource(schema.GroupVersionResource{
// 			Group:    rosBridgeServiceQuery.GroupVersionKind().Group,
// 			Version:  rosBridgeServiceQuery.GroupVersionKind().Version,
// 			Resource: "services",
// 		})
// 		svcWatcher, err := resourceInterface.Watch(ctx, metav1.ListOptions{
// 			FieldSelector: "metadata.name=" + instance.GetBridgeServiceMetadata().Name,
// 		})
// 		if err != nil {
// 			return err
// 		}

// 		defer svcWatcher.Stop()

// 		svcDeleted := false
// 		for {
// 			if !svcDeleted {
// 				select {
// 				case event := <-svcWatcher.ResultChan():

// 					if event.Type == watch.Deleted {
// 						logger.Info("FINALIZER: ROS bridge service is deleted gracefully.")
// 						svcDeleted = true
// 					}
// 				}
// 			} else {
// 				break
// 			}

// 		}
// 	}
// 	return nil
// }

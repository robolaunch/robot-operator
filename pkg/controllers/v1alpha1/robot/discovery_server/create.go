package discovery_server

import (
	"context"

	resources "github.com/robolaunch/robot-operator/internal/resources/v1alpha1"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/types"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *DiscoveryServerReconciler) createService(ctx context.Context, instance *robotv1alpha1.DiscoveryServer, svcNamespacedName *types.NamespacedName) error {

	svc := resources.GetDiscoveryServerService(instance, instance.GetDiscoveryServerServiceMetadata())

	err := ctrl.SetControllerReference(instance, svc, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, svc)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Service " + svc.Name + " is created.")
	return nil
}

func (r *DiscoveryServerReconciler) createPod(ctx context.Context, instance *robotv1alpha1.DiscoveryServer, podNamespacedName *types.NamespacedName) error {

	pod := resources.GetDiscoveryServerPod(instance, instance.GetDiscoveryServerPodMetadata())

	err := ctrl.SetControllerReference(instance, pod, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, pod)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Pod " + pod.Name + " is created.")
	return nil
}

func (r *DiscoveryServerReconciler) createConfigMap(ctx context.Context, instance *robotv1alpha1.DiscoveryServer, cmNamespacedName *types.NamespacedName) error {

	cm, err := resources.GetDiscoveryServerConfigMap(instance, instance.GetDiscoveryServerConfigMapMetadata())
	if err != nil {
		return err
	}

	err = ctrl.SetControllerReference(instance, cm, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, cm)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: ConfigMap " + cm.Name + " is created.")
	return nil
}

func (r *DiscoveryServerReconciler) createServiceExport(ctx context.Context, instance *robotv1alpha1.DiscoveryServer, seNamespacedName *types.NamespacedName) error {

	se, err := resources.GetDiscoveryServerServiceExport(instance, seNamespacedName)
	if err != nil {
		return err
	}

	err = ctrl.SetControllerReference(instance, se, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, se)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: ServiceExport " + se.Name + " is created.")
	return nil
}

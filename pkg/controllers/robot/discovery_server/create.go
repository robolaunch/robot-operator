package discovery_server

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/roboscale.io/v1alpha1"
	"github.com/robolaunch/robot-operator/internal/resources"
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
	if err != nil {
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
	if err != nil {
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
	if err != nil {
		return err
	}

	logger.Info("STATUS: ConfigMap " + cm.Name + " is created.")
	return nil
}

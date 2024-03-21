package edge_proxy

import (
	"context"

	v1alpha2_resources "github.com/robolaunch/robot-operator/internal/resources/v1alpha2"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *EdgeProxyReconciler) updateDeployment(ctx context.Context, instance *robotv1alpha2.EdgeProxy) error {

	node, err := r.reconcileGetNode(ctx, instance)
	if err != nil {
		return err
	}

	deployment := v1alpha2_resources.GetEdgeProxyDeployment(instance, instance.GetDeploymentMetadata(), *node)

	err = ctrl.SetControllerReference(instance, deployment, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Update(ctx, deployment)
	if err != nil {
		return err
	}

	r.Recorder.Event(instance, "Normal", "Updated", "Deployment '"+instance.GetDeploymentMetadata().Name+"' is updated to sync resources.")

	logger.Info("STATUS: Deployment " + deployment.Name + " is updated.")
	return nil
}

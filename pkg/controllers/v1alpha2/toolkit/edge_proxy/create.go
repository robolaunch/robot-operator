package edge_proxy

import (
	"context"
	"errors"

	v1alpha2_resources "github.com/robolaunch/robot-operator/internal/resources/v1alpha2"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	k8sErr "k8s.io/apimachinery/pkg/api/errors"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *EdgeProxyReconciler) createDeployment(ctx context.Context, instance *robotv1alpha2.EdgeProxy) error {

	node, err := r.reconcileGetNode(ctx, instance)
	if err != nil {
		return err
	}

	deployment := v1alpha2_resources.GetEdgeProxyDeployment(instance, instance.GetDeploymentMetadata(), *node)
	if deployment == nil {
		return errors.New("deployment is not valid")
	}

	err = ctrl.SetControllerReference(instance, deployment, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, deployment)
	if err != nil && k8sErr.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	r.Recorder.Event(instance, "Normal", "Created", "Deployment '"+instance.GetDeploymentMetadata().Name+"' is created.")

	return nil
}

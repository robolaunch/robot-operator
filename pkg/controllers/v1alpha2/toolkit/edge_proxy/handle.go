package edge_proxy

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"

	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

func (r *EdgeProxyReconciler) reconcileHandleDeployment(ctx context.Context, instance *robotv1alpha2.EdgeProxy) error {

	if !instance.Status.DeploymentStatus.Resource.Created {

		err := r.createDeployment(ctx, instance)
		if err != nil {
			return err
		}

		instance.Status.DeploymentStatus.Resource.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Deployment",
			ResourceName:      instance.GetDeploymentMetadata().Name,
			ResourceNamespace: instance.GetDeploymentMetadata().Namespace,
		}
	}

	return nil
}

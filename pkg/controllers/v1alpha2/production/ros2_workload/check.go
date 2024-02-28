package ros2_workload

import (
	"context"
	"reflect"

	"github.com/robolaunch/robot-operator/internal/reference"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *ROS2WorkloadReconciler) reconcileCheckDiscoveryServer(ctx context.Context, instance *robotv1alpha2.ROS2Workload) error {

	discoverServerQuery := &robotv1alpha1.DiscoveryServer{}
	err := r.Get(ctx, *instance.GetDiscoveryServerMetadata(), discoverServerQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.DiscoveryServerStatus = robotv1alpha1.DiscoveryServerInstanceStatus{}
	} else if err != nil {
		return err
	} else {

		if !reflect.DeepEqual(instance.Spec.DiscoveryServerTemplate, discoverServerQuery.Spec) {
			discoverServerQuery.Spec = instance.Spec.DiscoveryServerTemplate
			err = r.Update(ctx, discoverServerQuery)
			if err != nil {
				return err
			}
		}

		instance.Status.DiscoveryServerStatus.Resource.Created = true
		reference.SetReference(&instance.Status.DiscoveryServerStatus.Resource.Reference, discoverServerQuery.TypeMeta, discoverServerQuery.ObjectMeta)
		instance.Status.DiscoveryServerStatus.Status = discoverServerQuery.Status
	}

	return nil
}

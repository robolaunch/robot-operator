package robot_dev_suite

import (
	"context"
	"reflect"

	"github.com/robolaunch/robot-operator/internal/reference"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *RobotDevSuiteReconciler) reconcileCheckRobotVDI(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	robotVDIQuery := &robotv1alpha1.RobotVDI{}
	err := r.Get(ctx, *instance.GetRobotVDIMetadata(), robotVDIQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.RobotVDIStatus = robotv1alpha1.OwnedRobotServiceStatus{}
		} else {
			return err
		}
	} else {

		if instance.Spec.VDIEnabled {

			if !reflect.DeepEqual(instance.Spec.RobotVDITemplate, robotVDIQuery.Spec) {
				robotVDIQuery.Spec = instance.Spec.RobotVDITemplate
				err = r.Update(ctx, robotVDIQuery)
				if err != nil {
					return err
				}
			}

			instance.Status.RobotVDIStatus.Resource.Created = true
			reference.SetReference(&instance.Status.RobotVDIStatus.Resource.Reference, robotVDIQuery.TypeMeta, robotVDIQuery.ObjectMeta)
			instance.Status.RobotVDIStatus.Resource.Phase = string(robotVDIQuery.Status.Phase)
			instance.Status.RobotVDIStatus.Connection = robotVDIQuery.Status.ServiceTCPStatus.URL

		} else {

			err := r.Delete(ctx, robotVDIQuery)
			if err != nil {
				return err
			}

		}

	}

	return nil
}

func (r *RobotDevSuiteReconciler) reconcileCheckRobotIDE(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	robotIDEQuery := &robotv1alpha1.RobotIDE{}
	err := r.Get(ctx, *instance.GetRobotIDEMetadata(), robotIDEQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.RobotIDEStatus = robotv1alpha1.OwnedRobotServiceStatus{}
		} else {
			return err
		}
	} else {

		if instance.Spec.IDEEnabled {

			if !reflect.DeepEqual(instance.Spec.RobotIDETemplate, robotIDEQuery.Spec) {
				robotIDEQuery.Spec = instance.Spec.RobotIDETemplate
				err = r.Update(ctx, robotIDEQuery)
				if err != nil {
					return err
				}
			}

			instance.Status.RobotIDEStatus.Resource.Created = true
			reference.SetReference(&instance.Status.RobotIDEStatus.Resource.Reference, robotIDEQuery.TypeMeta, robotIDEQuery.ObjectMeta)
			instance.Status.RobotIDEStatus.Resource.Phase = string(robotIDEQuery.Status.Phase)
			instance.Status.RobotIDEStatus.Connection = robotIDEQuery.Status.ServiceStatus.URL

		} else {

			err := r.Delete(ctx, robotIDEQuery)
			if err != nil {
				return err
			}

		}

	}

	return nil
}

func (r *RobotDevSuiteReconciler) reconcileCheckRemoteIDERelayServer(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	remoteIDERelayServerQuery := &robotv1alpha1.RelayServer{}
	err := r.Get(ctx, *instance.GetRemoteIDERelayServerMetadata(), remoteIDERelayServerQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.RemoteIDERelayServerStatus = robotv1alpha1.OwnedRobotServiceStatus{}
		} else {
			return err
		}
	} else {

		if instance.Spec.RemoteIDEEnabled {

			if !reflect.DeepEqual(instance.Spec.RemoteIDERelayServerTemplate, remoteIDERelayServerQuery.Spec) {
				remoteIDERelayServerQuery.Spec = instance.Spec.RemoteIDERelayServerTemplate
				err = r.Update(ctx, remoteIDERelayServerQuery)
				if err != nil {
					return err
				}
			}

			instance.Status.RemoteIDERelayServerStatus.Resource.Created = true
			reference.SetReference(&instance.Status.RemoteIDERelayServerStatus.Resource.Reference, remoteIDERelayServerQuery.TypeMeta, remoteIDERelayServerQuery.ObjectMeta)
			instance.Status.RemoteIDERelayServerStatus.Resource.Phase = string(remoteIDERelayServerQuery.Status.Phase)
			instance.Status.RemoteIDERelayServerStatus.Connection = remoteIDERelayServerQuery.Status.ServiceStatus.URL

		} else {

			err := r.Delete(ctx, remoteIDERelayServerQuery)
			if err != nil {
				return err
			}

		}

	}

	return nil
}

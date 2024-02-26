package robot_dev_suite

import (
	"context"
	"time"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/api/errors"
	v1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

func (r *RobotDevSuiteReconciler) reconcileDeleteRobotVDI(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	robotVDIQuery := &robotv1alpha1.RobotVDI{}
	err := r.Get(ctx, *instance.GetRobotVDIMetadata(), robotVDIQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.RobotVDIStatus = robotv1alpha1.OwnedRobotServiceStatus{}
		} else {
			return err
		}
	} else {

		propagationPolicy := v1.DeletePropagationForeground
		err := r.Delete(ctx, robotVDIQuery, &client.DeleteOptions{
			PropagationPolicy: &propagationPolicy,
		})
		if err != nil {
			return err
		}

		// watch until it's deleted
		deleted := false
		for !deleted {
			robotVDIQuery := &robotv1alpha1.RobotVDI{}
			err := r.Get(ctx, *instance.GetRobotVDIMetadata(), robotVDIQuery)
			if err != nil && errors.IsNotFound(err) {
				deleted = true
			}
			time.Sleep(time.Second * 1)
		}

		instance.Status.RobotVDIStatus = robotv1alpha1.OwnedRobotServiceStatus{}
	}

	return nil
}

func (r *RobotDevSuiteReconciler) reconcileDeleteRobotIDE(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	robotIDEQuery := &robotv1alpha1.RobotIDE{}
	err := r.Get(ctx, *instance.GetRobotIDEMetadata(), robotIDEQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.RobotIDEStatus = robotv1alpha1.OwnedRobotServiceStatus{}
		} else {
			return err
		}
	} else {

		propagationPolicy := v1.DeletePropagationForeground
		err := r.Delete(ctx, robotIDEQuery, &client.DeleteOptions{
			PropagationPolicy: &propagationPolicy,
		})
		if err != nil {
			return err
		}

		// watch until it's deleted
		deleted := false
		for !deleted {
			robotIDEQuery := &robotv1alpha1.RobotIDE{}
			err := r.Get(ctx, *instance.GetRobotIDEMetadata(), robotIDEQuery)
			if err != nil && errors.IsNotFound(err) {
				deleted = true
			}
			time.Sleep(time.Second * 1)
		}

		instance.Status.RobotIDEStatus = robotv1alpha1.OwnedRobotServiceStatus{}
	}

	return nil
}

func (r *RobotDevSuiteReconciler) reconcileDeleteNotebook(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	notebookQuery := &robotv1alpha1.Notebook{}
	err := r.Get(ctx, *instance.GetNotebookMetadata(), notebookQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.NotebookStatus = robotv1alpha1.OwnedRobotServiceStatus{}
		} else {
			return err
		}
	} else {

		propagationPolicy := v1.DeletePropagationForeground
		err := r.Delete(ctx, notebookQuery, &client.DeleteOptions{
			PropagationPolicy: &propagationPolicy,
		})
		if err != nil {
			return err
		}

		// watch until it's deleted
		deleted := false
		for !deleted {
			notebookQuery := &robotv1alpha1.Notebook{}
			err := r.Get(ctx, *instance.GetNotebookMetadata(), notebookQuery)
			if err != nil && errors.IsNotFound(err) {
				deleted = true
			}
			time.Sleep(time.Second * 1)
		}

		instance.Status.NotebookStatus = robotv1alpha1.OwnedRobotServiceStatus{}
	}

	return nil
}

func (r *RobotDevSuiteReconciler) reconcileDeleteRemoteIDERelayServer(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	remoteIDERelayServerQuery := &robotv1alpha1.RelayServer{}
	err := r.Get(ctx, *instance.GetRemoteIDERelayServerMetadata(), remoteIDERelayServerQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.RemoteIDERelayServerStatus = robotv1alpha1.OwnedRobotServiceStatus{}
		} else {
			return err
		}
	} else {

		propagationPolicy := v1.DeletePropagationForeground
		err := r.Delete(ctx, remoteIDERelayServerQuery, &client.DeleteOptions{
			PropagationPolicy: &propagationPolicy,
		})
		if err != nil {
			return err
		}

		// watch until it's deleted
		deleted := false
		for !deleted {
			remoteIDERelayServerQuery := &robotv1alpha1.RelayServer{}
			err := r.Get(ctx, *instance.GetRemoteIDERelayServerMetadata(), remoteIDERelayServerQuery)
			if err != nil && errors.IsNotFound(err) {
				deleted = true
			}
			time.Sleep(time.Second * 1)
		}

		instance.Status.RemoteIDERelayServerStatus = robotv1alpha1.OwnedRobotServiceStatus{}
	}

	return nil
}

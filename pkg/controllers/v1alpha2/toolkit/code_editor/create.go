package code_editor

import (
	"context"

	v1alpha2_resources "github.com/robolaunch/robot-operator/internal/resources/v1alpha2"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	"k8s.io/apimachinery/pkg/api/errors"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *CodeEditorReconciler) createPersistentVolumeClaim(ctx context.Context, instance *robotv1alpha2.CodeEditor, key int) error {

	pvc := v1alpha2_resources.GetCodeEditorPersistentVolumeClaim(instance, instance.GetPersistentVolumeClaimMetadata(key), key)

	err := ctrl.SetControllerReference(instance, pvc, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, pvc)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	r.Recorder.Event(instance, "Normal", "Created", "PersistentVolumeClaim '"+instance.GetPersistentVolumeClaimMetadata(key).Name+"' is created.")

	logger.Info("STATUS: PVC " + instance.GetPersistentVolumeClaimMetadata(key).Name + " is created.")
	return nil
}

func (r *CodeEditorReconciler) createDeployment(ctx context.Context, instance *robotv1alpha2.CodeEditor) error {

	node, err := r.reconcileGetNode(ctx, instance)
	if err != nil {
		return err
	}

	deployment := v1alpha2_resources.GetCodeEditorDeployment(instance, instance.GetDeploymentMetadata(), *node)

	err = ctrl.SetControllerReference(instance, deployment, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, deployment)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	r.Recorder.Event(instance, "Normal", "Created", "Deployment '"+instance.GetDeploymentMetadata().Name+"' is created.")

	logger.Info("STATUS: Deployment " + instance.GetDeploymentMetadata().Name + " is created.")
	return nil
}

func (r *CodeEditorReconciler) createService(ctx context.Context, instance *robotv1alpha2.CodeEditor) error {

	service := v1alpha2_resources.GetCodeEditorService(instance, instance.GetServiceMetadata())

	err := ctrl.SetControllerReference(instance, service, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, service)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	r.Recorder.Event(instance, "Normal", "Created", "Service '"+instance.GetDeploymentMetadata().Name+"' is created.")

	logger.Info("STATUS: Service " + instance.GetServiceMetadata().Name + " is created.")
	return nil
}

func (r *CodeEditorReconciler) createIngress(ctx context.Context, instance *robotv1alpha2.CodeEditor) error {

	ingress := v1alpha2_resources.GetCodeEditorIngress(instance, instance.GetIngressMetadata())

	err := ctrl.SetControllerReference(instance, ingress, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, ingress)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	r.Recorder.Event(instance, "Normal", "Created", "Ingress '"+instance.GetDeploymentMetadata().Name+"' is created.")

	logger.Info("STATUS: Ingress " + instance.GetIngressMetadata().Name + " is created.")
	return nil
}

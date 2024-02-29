package notebook

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/label"
	resources "github.com/robolaunch/robot-operator/internal/resources/v1alpha1"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *NotebookReconciler) reconcileCreateService(ctx context.Context, instance *robotv1alpha1.Notebook) error {

	nbService := resources.GetNotebookService(instance, instance.GetNotebookServiceMetadata())

	err := ctrl.SetControllerReference(instance, nbService, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, nbService)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Notebook service is created.")

	return nil
}

func (r *NotebookReconciler) reconcileCreatePod(ctx context.Context, instance *robotv1alpha1.Notebook) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	robotVDI := &robotv1alpha1.RobotVDI{}
	if label.GetTargetRobotVDI(instance) != "" {
		robotVDI, err = r.reconcileGetTargetRobotVDI(ctx, instance)
		if err != nil {
			return err
		}
	}

	activeNode, err := r.reconcileCheckNode(ctx, robot)
	if err != nil {
		return err
	}

	cm := corev1.ConfigMap{}
	err = r.Get(ctx, *instance.GetNotebookConfigMapMetadata(), &cm)
	if err != nil {
		return err
	}

	nbPod := resources.GetNotebookPod(instance, instance.GetNotebookPodMetadata(), *robot, *robotVDI, *activeNode, cm)

	err = ctrl.SetControllerReference(instance, nbPod, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, nbPod)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Notebook pod is created.")

	return nil
}

func (r *NotebookReconciler) reconcileCreateIngress(ctx context.Context, instance *robotv1alpha1.Notebook) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	nbIngress := resources.GetNotebookIngress(instance, instance.GetNotebookIngressMetadata(), *robot)

	err = ctrl.SetControllerReference(instance, nbIngress, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, nbIngress)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Notebook ingress is created.")

	return nil
}

func (r *NotebookReconciler) reconcileCreateServiceExport(ctx context.Context, instance *robotv1alpha1.Notebook) error {

	nbServiceExport := resources.GetNotebookServiceExport(instance, instance.GetNotebookServiceExportMetadata())

	err := ctrl.SetControllerReference(instance, nbServiceExport, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, nbServiceExport)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Notebook service export is created.")

	return nil
}

func (r *NotebookReconciler) reconcileCreateCustomService(ctx context.Context, instance *robotv1alpha1.Notebook) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	nbService := resources.GetNotebookCustomService(instance, instance.GetNotebookCustomServiceMetadata(), *robot)

	err = ctrl.SetControllerReference(instance, nbService, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, nbService)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Notebook custom service is created.")

	return nil
}

func (r *NotebookReconciler) reconcileCreateCustomIngress(ctx context.Context, instance *robotv1alpha1.Notebook) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	nbIngress := resources.GetNotebookCustomIngress(instance, instance.GetNotebookCustomIngressMetadata(), *robot)

	err = ctrl.SetControllerReference(instance, nbIngress, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, nbIngress)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Notebook custom ingress is created.")

	return nil
}

func (r *NotebookReconciler) reconcileCreateConfigMap(ctx context.Context, instance *robotv1alpha1.Notebook) error {

	nbCm := resources.GetNotebookConfigMap(instance, instance.GetNotebookConfigMapMetadata())

	err := ctrl.SetControllerReference(instance, nbCm, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, nbCm)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Notebook config map is created.")

	return nil
}

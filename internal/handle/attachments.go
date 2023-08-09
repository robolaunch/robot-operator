package handle

import (
	"context"

	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/labels"
	"sigs.k8s.io/controller-runtime/pkg/client"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

func CheckIfAnyRDSActive(ctx context.Context, r client.Client, instance metav1.Object, robotSelector labels.Selector) error {
	robotDevSuiteList := robotv1alpha1.RobotDevSuiteList{}
	err := r.List(ctx, &robotDevSuiteList, &client.ListOptions{Namespace: instance.GetNamespace(), LabelSelector: robotSelector.Add()})
	if err != nil {
		return err
	}

	for _, rds := range robotDevSuiteList.Items {

		if rds.Status.Active {
			return &robotErr.RobotResourcesHasNotBeenReleasedError{
				ResourceName:      instance.GetName(),
				ResourceNamespace: instance.GetNamespace(),
			}
		}

		if rds.Status.Phase != robotv1alpha1.RobotDevSuitePhaseInactive {
			return &robotErr.RobotResourcesHasNotBeenReleasedError{
				ResourceName:      instance.GetName(),
				ResourceNamespace: instance.GetNamespace(),
			}
		}
	}

	return nil
}

func CheckIfAnyBMActive(ctx context.Context, r client.Client, instance metav1.Object, robotSelector labels.Selector) error {

	buildManagerList := robotv1alpha1.BuildManagerList{}
	err := r.List(ctx, &buildManagerList, &client.ListOptions{Namespace: instance.GetNamespace(), LabelSelector: robotSelector})
	if err != nil {
		return err
	}

	for _, bm := range buildManagerList.Items {

		if bm.Name == instance.GetName() {
			continue
		}

		if bm.Status.Active {
			return &robotErr.RobotResourcesHasNotBeenReleasedError{
				ResourceName:      instance.GetName(),
				ResourceNamespace: instance.GetNamespace(),
			}
		}

		if bm.Status.Phase != robotv1alpha1.BuildManagerInactive {
			return &robotErr.RobotResourcesHasNotBeenReleasedError{
				ResourceName:      instance.GetName(),
				ResourceNamespace: instance.GetNamespace(),
			}
		}
	}

	return nil
}

func CheckIfAnyLMActive(ctx context.Context, r client.Client, instance metav1.Object, robotSelector labels.Selector) error {

	launchManagerList := robotv1alpha1.LaunchManagerList{}
	err := r.List(ctx, &launchManagerList, &client.ListOptions{Namespace: instance.GetNamespace(), LabelSelector: robotSelector})
	if err != nil {
		return err
	}

	for _, lm := range launchManagerList.Items {

		if lm.Status.Active {
			return &robotErr.RobotResourcesHasNotBeenReleasedError{
				ResourceName:      instance.GetName(),
				ResourceNamespace: instance.GetNamespace(),
			}
		}

		if lm.Status.Phase != robotv1alpha1.LaunchManagerPhaseInactive {
			return &robotErr.RobotResourcesHasNotBeenReleasedError{
				ResourceName:      instance.GetName(),
				ResourceNamespace: instance.GetNamespace(),
			}
		}
	}

	return nil
}

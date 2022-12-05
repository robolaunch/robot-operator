package v1alpha1

import (
	"k8s.io/apimachinery/pkg/runtime"
	ctrl "sigs.k8s.io/controller-runtime"
	logf "sigs.k8s.io/controller-runtime/pkg/log"
	"sigs.k8s.io/controller-runtime/pkg/webhook"
)

// log is for logging in this package.
var robotlog = logf.Log.WithName("robot-resource")

func (r *Robot) SetupWebhookWithManager(mgr ctrl.Manager) error {
	return ctrl.NewWebhookManagedBy(mgr).
		For(r).
		Complete()
}

//+kubebuilder:webhook:path=/mutate-robot-roboscale-io-v1alpha1-robot,mutating=true,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=robots,verbs=create;update,versions=v1alpha1,name=mrobot.kb.io,admissionReviewVersions=v1

var _ webhook.Defaulter = &Robot{}

// Default implements webhook.Defaulter so a webhook will be registered for the type
func (r *Robot) Default() {
	robotlog.Info("default", "name", r.Name)

	// TODO(user): fill in your defaulting logic.
}

//+kubebuilder:webhook:path=/validate-robot-roboscale-io-v1alpha1-robot,mutating=false,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=robots,verbs=create;update,versions=v1alpha1,name=vrobot.kb.io,admissionReviewVersions=v1

var _ webhook.Validator = &Robot{}

// ValidateCreate implements webhook.Validator so a webhook will be registered for the type
func (r *Robot) ValidateCreate() error {
	robotlog.Info("validate create", "name", r.Name)
	return nil
}

// ValidateUpdate implements webhook.Validator so a webhook will be registered for the type
func (r *Robot) ValidateUpdate(old runtime.Object) error {
	robotlog.Info("validate update", "name", r.Name)
	return nil
}

// ValidateDelete implements webhook.Validator so a webhook will be registered for the type
func (r *Robot) ValidateDelete() error {
	robotlog.Info("validate delete", "name", r.Name)
	return nil
}

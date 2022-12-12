/*
Copyright 2022.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

package launch_manager

import (
	"context"

	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/dynamic"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
)

// LaunchManagerReconciler reconciles a LaunchManager object
type LaunchManagerReconciler struct {
	client.Client
	Scheme        *runtime.Scheme
	DynamicClient dynamic.Interface
}

//+kubebuilder:rbac:groups=robot.roboscale.io,resources=launchmanagers,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=launchmanagers/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.roboscale.io,resources=launchmanagers/finalizers,verbs=update

func (r *LaunchManagerReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	_ = log.FromContext(ctx)
	return ctrl.Result{}, nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *LaunchManagerReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.LaunchManager{}).
		Complete(r)
}

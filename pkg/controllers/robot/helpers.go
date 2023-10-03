package robot

import (
	"context"
	"errors"
	"strconv"
	"strings"

	"github.com/robolaunch/robot-operator/internal"
	robotErr "github.com/robolaunch/robot-operator/internal/error"
	label "github.com/robolaunch/robot-operator/internal/label"
	nodePkg "github.com/robolaunch/robot-operator/internal/node"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/apimachinery/pkg/selection"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/util/retry"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

func (r *RobotReconciler) reconcileGetInstance(ctx context.Context, meta types.NamespacedName) (*robotv1alpha1.Robot, error) {
	instance := &robotv1alpha1.Robot{}
	err := r.Get(ctx, meta, instance)
	if err != nil {
		return &robotv1alpha1.Robot{}, err
	}

	return instance, nil
}

func (r *RobotReconciler) reconcileUpdateInstanceStatus(ctx context.Context, instance *robotv1alpha1.Robot) error {
	return retry.RetryOnConflict(retry.DefaultRetry, func() error {
		instanceLV := &robotv1alpha1.Robot{}
		err := r.Get(ctx, types.NamespacedName{
			Name:      instance.Name,
			Namespace: instance.Namespace,
		}, instanceLV)

		if err == nil {
			instance.ResourceVersion = instanceLV.ResourceVersion
		}

		err1 := r.Status().Update(ctx, instance)
		return err1
	})
}

func (r *RobotReconciler) reconcileCheckNode(ctx context.Context, instance *robotv1alpha1.Robot) (*corev1.Node, error) {

	tenancyMap := label.GetTenancyMap(instance)

	requirements := []labels.Requirement{}
	for k, v := range tenancyMap {
		newReq, err := labels.NewRequirement(k, selection.In, []string{v})
		if err != nil {
			return nil, err
		}
		requirements = append(requirements, *newReq)
	}

	nodeSelector := labels.NewSelector().Add(requirements...)

	nodes := &corev1.NodeList{}
	err := r.List(ctx, nodes, &client.ListOptions{
		LabelSelector: nodeSelector,
	})
	if err != nil {
		return nil, err
	}

	if len(nodes.Items) == 0 {
		return nil, &robotErr.NodeNotFoundError{
			ResourceKind:      instance.Kind,
			ResourceName:      instance.Name,
			ResourceNamespace: instance.Namespace,
		}
	} else if len(nodes.Items) > 1 {
		return nil, &robotErr.MultipleNodeFoundError{
			ResourceKind:      instance.Kind,
			ResourceName:      instance.Name,
			ResourceNamespace: instance.Namespace,
		}
	}

	instance.Status.NodeName = nodes.Items[0].Name

	return &nodes.Items[0], nil
}

func (r *RobotReconciler) reconcileCheckImage(ctx context.Context, instance *robotv1alpha1.Robot) error {

	node, err := r.reconcileCheckNode(ctx, instance)
	if err != nil {
		return err
	}

	if instance.Status.Image == "" {
		instance.Status.Image, err = nodePkg.GetImage(ctx, r.Client, *node, *instance)
		if err != nil {
			return err
		}
	}

	return nil
}

func (r *RobotReconciler) reconcileCheckPersistentDirectories(ctx context.Context, instance *robotv1alpha1.Robot) error {

	if len(instance.Status.PersistentDirectories) == 0 {
		if dirsConfig, ok := instance.Spec.AdditionalConfigs[internal.PERSISTENT_DIRS_KEY]; ok {
			dirs := strings.Split(dirsConfig.Value, ":")

			// volumes for shared linux environment
			for k, dirPath := range dirs {
				instance.Status.PersistentDirectories = append(instance.Status.PersistentDirectories, robotv1alpha1.PersistentDirectory{
					Path: dirPath,
					Status: robotv1alpha1.OwnedResourceStatus{
						Reference: corev1.ObjectReference{
							Namespace: instance.Namespace,
							Name:      instance.Name + "-pvc-" + strconv.Itoa(k),
						},
					},
				})
			}

			// volume for workspace
			instance.Status.PersistentDirectories = append(instance.Status.PersistentDirectories, robotv1alpha1.PersistentDirectory{
				Path: instance.Spec.WorkspaceManagerTemplate.WorkspacesPath,
				Status: robotv1alpha1.OwnedResourceStatus{
					Reference: corev1.ObjectReference{
						Namespace: instance.Namespace,
						Name:      instance.Name + "-workspace",
					},
				},
			})
		} else {
			return errors.New("persistent directories should be specified in additional configs as PERSISTENT_DIRS")
		}
	}

	return nil
}

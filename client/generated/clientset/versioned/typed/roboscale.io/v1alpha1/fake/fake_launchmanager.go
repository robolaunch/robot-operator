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
// Code generated by client-gen. DO NOT EDIT.

package fake

import (
	"context"

	v1alpha1 "github.com/robolaunch/robot-operator/api/roboscale.io/v1alpha1"
	v1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	labels "k8s.io/apimachinery/pkg/labels"
	schema "k8s.io/apimachinery/pkg/runtime/schema"
	types "k8s.io/apimachinery/pkg/types"
	watch "k8s.io/apimachinery/pkg/watch"
	testing "k8s.io/client-go/testing"
)

// FakeLaunchManagers implements LaunchManagerInterface
type FakeLaunchManagers struct {
	Fake *FakeRoboscaleV1alpha1
	ns   string
}

var launchmanagersResource = schema.GroupVersionResource{Group: "roboscale.io", Version: "v1alpha1", Resource: "launchmanagers"}

var launchmanagersKind = schema.GroupVersionKind{Group: "roboscale.io", Version: "v1alpha1", Kind: "LaunchManager"}

// Get takes name of the launchManager, and returns the corresponding launchManager object, and an error if there is any.
func (c *FakeLaunchManagers) Get(ctx context.Context, name string, options v1.GetOptions) (result *v1alpha1.LaunchManager, err error) {
	obj, err := c.Fake.
		Invokes(testing.NewGetAction(launchmanagersResource, c.ns, name), &v1alpha1.LaunchManager{})

	if obj == nil {
		return nil, err
	}
	return obj.(*v1alpha1.LaunchManager), err
}

// List takes label and field selectors, and returns the list of LaunchManagers that match those selectors.
func (c *FakeLaunchManagers) List(ctx context.Context, opts v1.ListOptions) (result *v1alpha1.LaunchManagerList, err error) {
	obj, err := c.Fake.
		Invokes(testing.NewListAction(launchmanagersResource, launchmanagersKind, c.ns, opts), &v1alpha1.LaunchManagerList{})

	if obj == nil {
		return nil, err
	}

	label, _, _ := testing.ExtractFromListOptions(opts)
	if label == nil {
		label = labels.Everything()
	}
	list := &v1alpha1.LaunchManagerList{ListMeta: obj.(*v1alpha1.LaunchManagerList).ListMeta}
	for _, item := range obj.(*v1alpha1.LaunchManagerList).Items {
		if label.Matches(labels.Set(item.Labels)) {
			list.Items = append(list.Items, item)
		}
	}
	return list, err
}

// Watch returns a watch.Interface that watches the requested launchManagers.
func (c *FakeLaunchManagers) Watch(ctx context.Context, opts v1.ListOptions) (watch.Interface, error) {
	return c.Fake.
		InvokesWatch(testing.NewWatchAction(launchmanagersResource, c.ns, opts))

}

// Create takes the representation of a launchManager and creates it.  Returns the server's representation of the launchManager, and an error, if there is any.
func (c *FakeLaunchManagers) Create(ctx context.Context, launchManager *v1alpha1.LaunchManager, opts v1.CreateOptions) (result *v1alpha1.LaunchManager, err error) {
	obj, err := c.Fake.
		Invokes(testing.NewCreateAction(launchmanagersResource, c.ns, launchManager), &v1alpha1.LaunchManager{})

	if obj == nil {
		return nil, err
	}
	return obj.(*v1alpha1.LaunchManager), err
}

// Update takes the representation of a launchManager and updates it. Returns the server's representation of the launchManager, and an error, if there is any.
func (c *FakeLaunchManagers) Update(ctx context.Context, launchManager *v1alpha1.LaunchManager, opts v1.UpdateOptions) (result *v1alpha1.LaunchManager, err error) {
	obj, err := c.Fake.
		Invokes(testing.NewUpdateAction(launchmanagersResource, c.ns, launchManager), &v1alpha1.LaunchManager{})

	if obj == nil {
		return nil, err
	}
	return obj.(*v1alpha1.LaunchManager), err
}

// UpdateStatus was generated because the type contains a Status member.
// Add a +genclient:noStatus comment above the type to avoid generating UpdateStatus().
func (c *FakeLaunchManagers) UpdateStatus(ctx context.Context, launchManager *v1alpha1.LaunchManager, opts v1.UpdateOptions) (*v1alpha1.LaunchManager, error) {
	obj, err := c.Fake.
		Invokes(testing.NewUpdateSubresourceAction(launchmanagersResource, "status", c.ns, launchManager), &v1alpha1.LaunchManager{})

	if obj == nil {
		return nil, err
	}
	return obj.(*v1alpha1.LaunchManager), err
}

// Delete takes name of the launchManager and deletes it. Returns an error if one occurs.
func (c *FakeLaunchManagers) Delete(ctx context.Context, name string, opts v1.DeleteOptions) error {
	_, err := c.Fake.
		Invokes(testing.NewDeleteActionWithOptions(launchmanagersResource, c.ns, name, opts), &v1alpha1.LaunchManager{})

	return err
}

// DeleteCollection deletes a collection of objects.
func (c *FakeLaunchManagers) DeleteCollection(ctx context.Context, opts v1.DeleteOptions, listOpts v1.ListOptions) error {
	action := testing.NewDeleteCollectionAction(launchmanagersResource, c.ns, listOpts)

	_, err := c.Fake.Invokes(action, &v1alpha1.LaunchManagerList{})
	return err
}

// Patch applies the patch and returns the patched launchManager.
func (c *FakeLaunchManagers) Patch(ctx context.Context, name string, pt types.PatchType, data []byte, opts v1.PatchOptions, subresources ...string) (result *v1alpha1.LaunchManager, err error) {
	obj, err := c.Fake.
		Invokes(testing.NewPatchSubresourceAction(launchmanagersResource, c.ns, name, pt, data, subresources...), &v1alpha1.LaunchManager{})

	if obj == nil {
		return nil, err
	}
	return obj.(*v1alpha1.LaunchManager), err
}
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
// Code generated by lister-gen. DO NOT EDIT.

package v1alpha1

import (
	v1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/client-go/tools/cache"
)

// ROSBridgeLister helps list ROSBridges.
// All objects returned here must be treated as read-only.
type ROSBridgeLister interface {
	// List lists all ROSBridges in the indexer.
	// Objects returned here must be treated as read-only.
	List(selector labels.Selector) (ret []*v1alpha1.ROSBridge, err error)
	// ROSBridges returns an object that can list and get ROSBridges.
	ROSBridges(namespace string) ROSBridgeNamespaceLister
	ROSBridgeListerExpansion
}

// rOSBridgeLister implements the ROSBridgeLister interface.
type rOSBridgeLister struct {
	indexer cache.Indexer
}

// NewROSBridgeLister returns a new ROSBridgeLister.
func NewROSBridgeLister(indexer cache.Indexer) ROSBridgeLister {
	return &rOSBridgeLister{indexer: indexer}
}

// List lists all ROSBridges in the indexer.
func (s *rOSBridgeLister) List(selector labels.Selector) (ret []*v1alpha1.ROSBridge, err error) {
	err = cache.ListAll(s.indexer, selector, func(m interface{}) {
		ret = append(ret, m.(*v1alpha1.ROSBridge))
	})
	return ret, err
}

// ROSBridges returns an object that can list and get ROSBridges.
func (s *rOSBridgeLister) ROSBridges(namespace string) ROSBridgeNamespaceLister {
	return rOSBridgeNamespaceLister{indexer: s.indexer, namespace: namespace}
}

// ROSBridgeNamespaceLister helps list and get ROSBridges.
// All objects returned here must be treated as read-only.
type ROSBridgeNamespaceLister interface {
	// List lists all ROSBridges in the indexer for a given namespace.
	// Objects returned here must be treated as read-only.
	List(selector labels.Selector) (ret []*v1alpha1.ROSBridge, err error)
	// Get retrieves the ROSBridge from the indexer for a given namespace and name.
	// Objects returned here must be treated as read-only.
	Get(name string) (*v1alpha1.ROSBridge, error)
	ROSBridgeNamespaceListerExpansion
}

// rOSBridgeNamespaceLister implements the ROSBridgeNamespaceLister
// interface.
type rOSBridgeNamespaceLister struct {
	indexer   cache.Indexer
	namespace string
}

// List lists all ROSBridges in the indexer for a given namespace.
func (s rOSBridgeNamespaceLister) List(selector labels.Selector) (ret []*v1alpha1.ROSBridge, err error) {
	err = cache.ListAllByNamespace(s.indexer, s.namespace, selector, func(m interface{}) {
		ret = append(ret, m.(*v1alpha1.ROSBridge))
	})
	return ret, err
}

// Get retrieves the ROSBridge from the indexer for a given namespace and name.
func (s rOSBridgeNamespaceLister) Get(name string) (*v1alpha1.ROSBridge, error) {
	obj, exists, err := s.indexer.GetByKey(s.namespace + "/" + name)
	if err != nil {
		return nil, err
	}
	if !exists {
		return nil, errors.NewNotFound(v1alpha1.Resource("rosbridge"), name)
	}
	return obj.(*v1alpha1.ROSBridge), nil
}

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

// RobotVDILister helps list RobotVDIs.
// All objects returned here must be treated as read-only.
type RobotVDILister interface {
	// List lists all RobotVDIs in the indexer.
	// Objects returned here must be treated as read-only.
	List(selector labels.Selector) (ret []*v1alpha1.RobotVDI, err error)
	// RobotVDIs returns an object that can list and get RobotVDIs.
	RobotVDIs(namespace string) RobotVDINamespaceLister
	RobotVDIListerExpansion
}

// robotVDILister implements the RobotVDILister interface.
type robotVDILister struct {
	indexer cache.Indexer
}

// NewRobotVDILister returns a new RobotVDILister.
func NewRobotVDILister(indexer cache.Indexer) RobotVDILister {
	return &robotVDILister{indexer: indexer}
}

// List lists all RobotVDIs in the indexer.
func (s *robotVDILister) List(selector labels.Selector) (ret []*v1alpha1.RobotVDI, err error) {
	err = cache.ListAll(s.indexer, selector, func(m interface{}) {
		ret = append(ret, m.(*v1alpha1.RobotVDI))
	})
	return ret, err
}

// RobotVDIs returns an object that can list and get RobotVDIs.
func (s *robotVDILister) RobotVDIs(namespace string) RobotVDINamespaceLister {
	return robotVDINamespaceLister{indexer: s.indexer, namespace: namespace}
}

// RobotVDINamespaceLister helps list and get RobotVDIs.
// All objects returned here must be treated as read-only.
type RobotVDINamespaceLister interface {
	// List lists all RobotVDIs in the indexer for a given namespace.
	// Objects returned here must be treated as read-only.
	List(selector labels.Selector) (ret []*v1alpha1.RobotVDI, err error)
	// Get retrieves the RobotVDI from the indexer for a given namespace and name.
	// Objects returned here must be treated as read-only.
	Get(name string) (*v1alpha1.RobotVDI, error)
	RobotVDINamespaceListerExpansion
}

// robotVDINamespaceLister implements the RobotVDINamespaceLister
// interface.
type robotVDINamespaceLister struct {
	indexer   cache.Indexer
	namespace string
}

// List lists all RobotVDIs in the indexer for a given namespace.
func (s robotVDINamespaceLister) List(selector labels.Selector) (ret []*v1alpha1.RobotVDI, err error) {
	err = cache.ListAllByNamespace(s.indexer, s.namespace, selector, func(m interface{}) {
		ret = append(ret, m.(*v1alpha1.RobotVDI))
	})
	return ret, err
}

// Get retrieves the RobotVDI from the indexer for a given namespace and name.
func (s robotVDINamespaceLister) Get(name string) (*v1alpha1.RobotVDI, error) {
	obj, exists, err := s.indexer.GetByKey(s.namespace + "/" + name)
	if err != nil {
		return nil, err
	}
	if !exists {
		return nil, errors.NewNotFound(v1alpha1.Resource("robotvdi"), name)
	}
	return obj.(*v1alpha1.RobotVDI), nil
}

#!/usr/bin/env python

'''
                   GNU LESSER GENERAL PUBLIC LICENSE
                       Version 3, 29 June 2007

 Copyright (C) 2007 Free Software Foundation, Inc. <http://fsf.org/>
 Everyone is permitted to copy and distribute verbatim copies
 of this license document, but changing it is not allowed.


  This version of the GNU Lesser General Public License incorporates
the terms and conditions of version 3 of the GNU General Public
License, supplemented by the additional permissions listed below.

  0. Additional Definitions.

  As used herein, "this License" refers to version 3 of the GNU Lesser
General Public License, and the "GNU GPL" refers to version 3 of the GNU
General Public License.

  "The Library" refers to a covered work governed by this License,
other than an Application or a Combined Work as defined below.

  An "Application" is any work that makes use of an interface provided
by the Library, but which is not otherwise based on the Library.
Defining a subclass of a class defined by the Library is deemed a mode
of using an interface provided by the Library.

  A "Combined Work" is a work produced by combining or linking an
Application with the Library.  The particular version of the Library
with which the Combined Work was made is also called the "Linked
Version".

  The "Minimal Corresponding Source" for a Combined Work means the
Corresponding Source for the Combined Work, excluding any source code
for portions of the Combined Work that, considered in isolation, are
based on the Application, and not on the Linked Version.

  The "Corresponding Application Code" for a Combined Work means the
object code and/or source code for the Application, including any data
and utility programs needed for reproducing the Combined Work from the
Application, but excluding the System Libraries of the Combined Work.

  1. Exception to Section 3 of the GNU GPL.

  You may convey a covered work under sections 3 and 4 of this License
without being bound by section 3 of the GNU GPL.

  2. Conveying Modified Versions.

  If you modify a copy of the Library, and, in your modifications, a
facility refers to a function or data to be supplied by an Application
that uses the facility (other than as an argument passed when the
facility is invoked), then you may convey a copy of the modified
version:

   a) under this License, provided that you make a good faith effort to
   ensure that, in the event an Application does not supply the
   function or data, the facility still operates, and performs
   whatever part of its purpose remains meaningful, or

   b) under the GNU GPL, with none of the additional permissions of
   this License applicable to that copy.

  3. Object Code Incorporating Material from Library Header Files.

  The object code form of an Application may incorporate material from
a header file that is part of the Library.  You may convey such object
code under terms of your choice, provided that, if the incorporated
material is not limited to numerical parameters, data structure
layouts and accessors, or small macros, inline functions and templates
(ten or fewer lines in length), you do both of the following:

   a) Give prominent notice with each copy of the object code that the
   Library is used in it and that the Library and its use are
   covered by this License.

   b) Accompany the object code with a copy of the GNU GPL and this license
   document.

  4. Combined Works.

  You may convey a Combined Work under terms of your choice that,
taken together, effectively do not restrict modification of the
portions of the Library contained in the Combined Work and reverse
engineering for debugging such modifications, if you also do each of
the following:

   a) Give prominent notice with each copy of the Combined Work that
   the Library is used in it and that the Library and its use are
   covered by this License.

   b) Accompany the Combined Work with a copy of the GNU GPL and this license
   document.

   c) For a Combined Work that displays copyright notices during
   execution, include the copyright notice for the Library among
   these notices, as well as a reference directing the user to the
   copies of the GNU GPL and this license document.

   d) Do one of the following:

       0) Convey the Minimal Corresponding Source under the terms of this
       License, and the Corresponding Application Code in a form
       suitable for, and under terms that permit, the user to
       recombine or relink the Application with a modified version of
       the Linked Version to produce a modified Combined Work, in the
       manner specified by section 6 of the GNU GPL for conveying
       Corresponding Source.

       1) Use a suitable shared library mechanism for linking with the
       Library.  A suitable mechanism is one that (a) uses at run time
       a copy of the Library already present on the user's computer
       system, and (b) will operate properly with a modified version
       of the Library that is interface-compatible with the Linked
       Version.

   e) Provide Installation Information, but only if you would otherwise
   be required to provide such information under section 6 of the
   GNU GPL, and only to the extent that such information is
   necessary to install and execute a modified version of the
   Combined Work produced by recombining or relinking the
   Application with a modified version of the Linked Version. (If
   you use option 4d0, the Installation Information must accompany
   the Minimal Corresponding Source and Corresponding Application
   Code. If you use option 4d1, you must provide the Installation
   Information in the manner specified by section 6 of the GNU GPL
   for conveying Corresponding Source.)

  5. Combined Libraries.

  You may place library facilities that are a work based on the
Library side by side in a single library together with other library
facilities that are not Applications and are not covered by this
License, and convey such a combined library under terms of your
choice, if you do both of the following:

   a) Accompany the combined library with a copy of the same work based
   on the Library, uncombined with any other library facilities,
   conveyed under the terms of this License.

   b) Give prominent notice with the combined library that part of it
   is a work based on the Library, and explaining where to find the
   accompanying uncombined form of the same work.

  6. Revised Versions of the GNU Lesser General Public License.

  The Free Software Foundation may publish revised and/or new versions
of the GNU Lesser General Public License from time to time. Such new
versions will be similar in spirit to the present version, but may
differ in detail to address new problems or concerns.

  Each version is given a distinguishing version number. If the
Library as you received it specifies that a certain numbered version
of the GNU Lesser General Public License "or any later version"
applies to it, you have the option of following the terms and
conditions either of that published version or of any later version
published by the Free Software Foundation. If the Library as you
received it does not specify a version number of the GNU Lesser
General Public License, you may choose any version of the GNU Lesser
General Public License ever published by the Free Software Foundation.

  If the Library as you received it specifies that a proxy can decide
whether future versions of the GNU Lesser General Public License shall
apply, that proxy's public statement of acceptance of any version is
permanent authorization for you to choose that version for the
Library.
'''

from Exp import Exp
from CondExp import CondExp
from Exp import Exp
from Fluents import Fluents


class Action:
    """
    This class implements a representation of an action
    attributes :
    - name
    - duration
    - preconditions
    - effects
    """
    def __init__(self, name=None, parameters=None, preconditions=None, cond_expression=None, cost=1.0):
        """
        Constructor of the Class
        :param name: the name of the action represented by str
        :param parameters: the parameters of the actions represented by a list of str
        :param preconditions: the precondition of the action represented by a Exp object
        :param cond_expression: the condition_expression of the action represented by a CondExp object
        :param cost: the cost of the action represented by int
        """
        # If we only know the name
        if name is not None \
                and parameters is None \
                and preconditions is None \
                and cond_expression is None \
                and isinstance(name, str):
            self._name = name
            self._cost = cost

        # If we know the name and the parameters (str)
        elif name is not None \
                and parameters is not None \
                and isinstance(parameters, str)\
                and preconditions is None \
                and cond_expression is None \
                and isinstance(name, str):
            self._parameters = parameters
            self._name = name
            self._cost = cost

        # If we know the name and the parameters  (list)
        elif name is not None \
                and parameters is not None \
                and isinstance(parameters, list) \
                and preconditions is None \
                and cond_expression is None \
                and isinstance(name, str):

            self._parameters = list()
            self._parameters = parameters
            self._name = name
            self._cost = cost

        # If we know the name, the parameters (str), the preconditions and the cond_expression
        elif name is not None \
                and preconditions is not None \
                and cond_expression is not None \
                and parameters is not None \
                and Action.is_cond_exp_list(cond_expression) \
                and isinstance(preconditions, Exp) \
                and isinstance(name, str) \
                and isinstance(parameters, str):

            self._name = name
            self._preconditions = preconditions
            self._parameters = parameters
            self._cost = cost
            self._cond_expression = list()

            for elem in cond_expression:
                self._cond_expression.append(elem)

        # If we know the nale, the paramters (list), the preconditions and the cond_expression
        elif name is not None \
                and preconditions is not None \
                and cond_expression is not None \
                and parameters is not None \
                and Action.is_cond_exp_list(cond_expression) \
                and isinstance(preconditions, Exp) \
                and isinstance(name, str) \
                and isinstance(parameters, list):

            self._name = name
            self._preconditions = preconditions
            self._parameters = list()
            self._cost = cost

            if isinstance(parameters,list):
                for elem in parameters:
                    self._parameters.append(elem)

            self._cond_expression = list()
            for elem in cond_expression:
                self._cond_expression.append(elem)

        # If none of that we can't create an action
        else:
            raise SyntaxError("Error in the function __init__ in class Action.py, the object was not created")

    @staticmethod
    def is_cond_exp_list(liste):
        """
        Return <code>True</code> if the list in parameter contains only CondExp objects, <code>False</code> otherwise
        :param liste: The list of CondExp
        :return: if the List is composed only of CondExp elements
        """
        bool_test = False
        if isinstance(liste, list):
            for elem in liste:
                if not isinstance(elem, CondExp):
                    bool_test = True
                    break
        elif isinstance(liste, CondExp):
            bool_test = True

        return bool

    def _get_name(self):
        """
        Action name's getter
        :return: The action name
        """
        return self._name

    def _get_preconditions(self):
        """
        Precondition's getter
        :return: The precondtion which is a Exp named self._precondtions
        """
        return self._preconditions

    def _get_cond_expressions(self):
        """
        Effects's getter
        :return: The list of CondExp named self._cond_expression
        """
        return self._cond_expression

    def _get_parameters(self):
        """
        Parameters's getter
        :return: The precondtion which is a Exp named self.parameters
        """
        return self._parameters

    def _get_cost(self):
        """
        Action cost's getter
        :return: the cost of the Action
        """
        return self._cost

    def _set_cost(self, cost):
        """
        Actions cost's setter
        :param cost: the cost of the Action
        """
        if isinstance(cost, int):
            self._cost = cost
        else:
            raise SyntaxError("Error in function _set_cost in class Action.py, the value of the parameter is not an int")

    def _set_name(self, name):
        """
        Actions name's setter
        :param name: the name of the Action
        :return:
        """
        if isinstance(name, str):
            self._name = name
        else:
            raise SyntaxError("Error in function _set_name in class Action.py, the value of the parameter is not a str")

    def _set_parameters(self, parameters):
        """
        Parameters's setter
        :param parameters: The Exp object we want to set in the Action
        """
        if isinstance(parameters,str):
            self._preconditions = parameters
        elif isinstance(parameters, list):
            for elem in parameters:
                self._parameters.append(elem)
        else:
            raise SyntaxError("Error in function _set_parameters in class Action.py, the value of the parameter "
                              "is not a str")

    def _set_preconditions(self, preconditions):
        """
        Preconditions's setter
        :param preconditions: The Exp object we want to set in the Action
        """
        if isinstance(preconditions, Exp):
            self._preconditions = preconditions
        else:
            SyntaxError("Error in function _set_precondition in class Action.py, the value of the "
                        "parameter is not a Exp")

    def _set_cond_expressions(self, cond_expression):
        """
        Effects's setter
        :param cond_expression: The list of CondExp that we want to set in the object
        """
        if Action.is_cond_exp_list(cond_expression):
            self._cond_expression = cond_expression
        else:
            SyntaxError("Error in function _set_cond_expressions in class Action.py, the value of the parameter "
                        "is not a list of cond_expression")

    def add_condition_exp(self, effects):
        """
        Add a CondExp to the list of CondExp of the action
        :param effects: the CondExp we want to add
        :return:
        """
        if isinstance(effects, list):
            if Action.is_cond_exp_list(effects):
                for elem in effects:
                    self._cond_expression.append(elem)
                    return True
        elif isinstance(effects,CondExp):
            if isinstance(self._cond_expression, list):
                self._cond_expression.append(effects)
                return True
            else:
                save = self._cond_expression
                self._cond_expression = list()
                self._cond_expression.append(save)
                self._cond_expression.append(effects)
                return True
        else:
            raise SyntaxError("Eror in function add_condition_exp in class Action.py, the parameter is not a list of "
                              "cond_expression or a cond_expression the cond_expression list was not add")

    def affiche(self):
        """
        This function display the informations of the Action
        """
        if isinstance(self._parameters, str):
            print ("Display of the Action : " + self._name + " : " + self._get_parameters() + "\n")
        elif isinstance(self._parameters, list):
            liste_affiche = ', '.join(self._parameters)
            print ("Display of the Action :  " + self._name + " : " + liste_affiche + "\n")
        else:
            raise SyntaxError("Error in function affiche in class Action.py, Can't display the parameters of the "
                              "action, error in conception")

        print ("Les preconditions : \n")
        self.preconditions.affiche()

        print ("\n La liste de CondExp : \n")
        for elem in self._cond_expression:
            elem.affiche()
        print ("\n")

    # This is the properties, that protect the use of the _gets and _sets of the class
    name = property(_get_name, _set_name)
    parameters = property(_get_parameters, _set_parameters)
    cost = property(_get_cost, _set_cost)
    preconditions = property(_get_preconditions, _set_preconditions)
    effects = property(_get_cond_expressions,_set_cond_expressions)

if __name__ == '__main__':

    # Test of the Class Action

    fluents_positives_exp = Fluents("on", "a", "b")
    fluents_negatives_exp = Fluents("handempty")

    exp = Exp(fluents_positives_exp, fluents_negatives_exp)

    fluents_positives_exp_cond_exp_1 = Fluents("on", "c", "d")
    fluents_negatives_exp_cond_exp_1 = Fluents("on", "a", "g")
    fluents_positives_exp_cond_exp_2 = Fluents("on", "e", "f")
    fluents_negatives_exp_cond_exp_2 = Fluents("on", "b", "h")

    exp_cond_exp_1 = Exp(fluents_positives_exp_cond_exp_1, fluents_negatives_exp_cond_exp_1)
    exp_cond_exp_2 = Exp(fluents_positives_exp_cond_exp_2, fluents_negatives_exp_cond_exp_2)

    cond_exp = CondExp(exp_cond_exp_1,exp_cond_exp_2)

    fluents_positives_action = Fluents("on", "g", "h")
    fluents_negatives_action = Fluents("on", "i", "j")

    exp_action = Exp(fluents_positives_action, fluents_negatives_action)

    action = Action("unstack", ["a", "b"], exp_action, cond_exp)

    fluents_positives_add_1 = Fluents("on", "n", "v")
    fluents_negatives_add_2 = Fluents("on", "w", "x")
    fluents_positives_add_3 = Fluents("on", "h", "y")
    fluents_negatives_add_4 = Fluents("on", "l", "m")

    exp_add_1 = Exp(fluents_positives_add_1, fluents_negatives_add_2)
    exp_add_2 = Exp(fluents_positives_add_3, fluents_negatives_add_4)

    cond_exp_add = CondExp(exp_add_1, exp_add_2)

    action.add_condition_exp(cond_exp_add)
    action.affiche()

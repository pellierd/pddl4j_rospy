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

from AbstractPlan import AbstractPlan
from Action import Action
from copy import copy


class SequentialPlan (AbstractPlan):
    """
    This class allow us to use Sequential plan
    """
    def __init__(self, actions=None):
        """
        Constructor of the class
        :param actions: a lsit of actions
        """
        super
        if isinstance(actions, list) and SequentialPlan.is_list_action(actions):
            self._actions = list()

            for elem in actions:
                self._actions.append(elem)
        else:
            self._actions = list()

    @staticmethod
    def is_list_action(liste_actions):
        """
        return if the liste contains only actions
        :param liste_actions:
        :return: <code>True</code> if the list contains only actions
        """
        bool_is_liste_action = True
        for elem in liste_actions:
            if not isinstance(elem,Action):
                bool_is_liste_action = False
        return bool_is_liste_action

    def makespan(self):
        """
        Returns the makespan of the plan. The makespan of a sequential plan is its size.
        :return: the makespan of the plan.
        see Plan#size()
        """
        return self.size()

    def actions(self):
        """
        Returns the list of actions contained in the plan ordered depending on their time specifier

        :return: the ordered set of actions of the plan.
        see Plan#actions()
        """
        return self._actions

    def time_specifiers(self):
        """
        Returns the set of time specifiers used in this plan.
        :return: the set of time specifiers used in this plan.
        see Plan#timeSpecifiers()
        """
        ts = list()
        for i in range(0, len(self._actions)):
            ts.append(i)

        return ts

    def get_actions_set(self, time):
        """
        Returns the set of actions at a specified time specifier.
        :param time: the time specifier.
        :return: the set of actions at a specified time specifier or null if no actions are scheduled in the plan at the
        the time specifier.
        see Plan#get_action_set(int)
        """
        if isinstance(time, int):
            if self.is_time_specifiers_out_of_bound(time):
                return None
            ts = list()
            ts.append(self._actions[time])
            return ts
        else:
            raise SyntaxError("Error in function get_actions_set in class Sequential_plan.py,"
                              "the value of the parameter is not a int")

    def add(self, time, action):
        """
        Adds an action at a specified time specifier in the plan.
        :param time: the action to add.
        :param action: the time specifier of the action in the plan.
        :return: <code>true</code> if the action was added; <code>false</code> otherwise.
        see Plan#add(int, Action)
        """
        if isinstance(time, int):
            if self.is_time_specifiers_out_of_bound(time):
                return False
        self._actions[int(time)] = action

    def remove(self, time, action=None):
        """
        :param time:the time specifier of the action in the plan to remove.
        :param action:the action to remove.
        :return: <code>true</code> if the action was added; <code>false</code> otherwise.
        see Plan#remove(int, time, action)
        """
        if isinstance(action, Action) \
                and self.get_actions_set(time)[0].name == action.name and isinstance(time, int) \
                and not self.is_time_specifiers_out_of_bound(time):
            self._actions.pop(time)
            return True
        elif action is None and isinstance(time,int) and not self.is_time_specifiers_out_of_bound(time):
            self._actions.pop(time)
            return True
        else:
            return False

    def contains(self, time, action):
        """
        Returns if an action is contained in the plan at a specified time specifier.
        :param time: the time specifier.
        :param action: the action.
        :return: <code>true</code> if the specified action is contained in the plan at the specified time specifier;
         <code>false</code> otherwise.
        """
        return not(self.is_time_specifiers_out_of_bound(time) and self._actions[time])

    def clear(self):
        """
        Removes all the actions of the plan.
        """
        self._actions.clear()

    def equals(self, obj):
        """
        Returns if the plan is equal to an other object. A plan is equal to an other object if the object is an instance
        of the same class and have the same action at the same time specifier.
        :param obj: the object to be compared.
        :return: <code>true</code> if this plan is equal to the specified object; <code>false</code> otherwise.
        """
        if obj is not None and isinstance(obj, SequentialPlan) and self._actions == obj.actions:
            return True
        else:
            return False

    def is_time_specifiers_out_of_bound(self, time):
        """
        Returns if a specified time specifier is invalid. Formally a time specifier is invalid if it is less
        than 0 or greater than the size of the plan.
        :param time: the time stamp.
        :return: <code>true</code> if the specified time specifier is out of bound; <code>false</code> otherwise.
        """
        if isinstance(time, int):
            return time < 0 or time > len(self.actions())


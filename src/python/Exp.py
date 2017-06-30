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

import copy
from Fluents import Fluents


class Exp:
    """
    This class implements an expression. An expression is used to encode preconditions
    (negative and positive) but also effects of the instantiated action.
    """

    def __init__(self, positives=None, negatives=None):
        """
            This is the construtor of our class, you can use it without instanciate
            the positives or the negatives or you can create one with predefined
            positives and negatives lists
        """
        if positives is not None \
                and negatives is not None \
                and isinstance(positives, list) \
                and isinstance(negatives, list):
            error = False
            for elemp in positives:
                if not isinstance(elemp, Fluents):
                    error = True
            for elemn in negatives:
                if not isinstance(elemn, Fluents):
                    error = True
            if not error:
                self._positives = positives
                self._negatives = negatives
            else:
                print ("Erreur un des elements de la liste n'est pas un Fluent")
        elif positives is not None \
                and negatives is not None \
                and isinstance(positives, Fluents) \
                and isinstance(negatives, Fluents):
            self._positives = list()
            self._negatives = list()

            self._positives.append(positives)
            self._negatives.append(negatives)
        else:
            self._positives = []
            self._negatives = []

    def copy(self):
        """
        This will act as the copy constructor.
        Work like that : b = a.copy()
        if the object is none then it will create one
        :return: the object
        """
        if self is None:
            return Exp()
        else:
            return copy.deepcopy(self)

    def _get_positives(self):
        """
        return the list of positives
        """
        return self._positives

    def _get_negatives(self):
        """
        return the list of negatives
        """
        return self._negatives

    def _set_positives(self, positives):
        """
        Positives's setter
            :param positives : the list we want to set for the positives
        """
        if isinstance(positives, Fluents):
            self._positives = positives
        else:
            print("L'argument pour le set positives n'est pas un Fluent, veuillez reesayer")

    def _set_negatives(self,negatives):
        """
        Negatives's setter
            :param negatives : the list we want to set for the negatives
        """
        if isinstance(negatives,Fluents):
            self._negatives = negatives
        else:
            print("L'argument pour le set negatives n'est pas un Fluent, veuillez reesayer")

    def add_one_positive(self, positive):
        """
        Add a positive to the list of positives
        :param positive ; a Fluent that we want to add to the list of positives
        """
        if isinstance(positive, Fluents):
            self._positives.append(positive)
        else:
            print("L'argument pour l'ajout n'est pas un fluent, veuillez reesayer")

    def add_positives(self, *positives):
        """
        Add positives to the list of positives
        :param positives : a list of positives that we want to add to the list of positives
        """
        i = 0
        for elem in positives:
            if isinstance(elem, Fluents):
                self._positives.append(elem)
            else:
                print("L'argument numero " + repr(i) + " n'est pas un fluent, il n'as donc pas ete ajoute ")

    def add_one_negative(self, negative):
        """
        Add a negative to the list of negatives
        :param negative : a Fluent that we want to add to the list of negatives
        """
        if isinstance(negative, Fluents):
            self._negatives.append(negative)
            self._negatives.append(negative)
        else:
            print("L'argument pour l'ajout n'est pas un fluent, veuillez reesayer")

    def add_negatives(self, *negatives):
        """
        Add negatives to the list of negatives
        :param negatives : a list of negatives that we want to add to the list of negatives
        """
        i = 0
        for elem in negatives:
            if isinstance(elem, Fluents):
                self._negatives.append(elem)
            else:
                print("L'argument numero " + repr(i) + " n'est pas un fluent, il n'as donc pas ete ajoute ")

    def is_empty(self):
        """
        Check if the lists are empty
        :return: True or False
        """
        if not self._positives and not self._negatives:
            return True
        else:
            return False

    def affiche(self):
        """
        Display the informations of the object
        """
        print ("Affichage de l'Exp \n")
        print ("Affichage des positives : ")
        for elem in self._positives:
            print ("----------------------")
            elem.affiche_fluent()

        print ("\n")
        print ("Affichage des negatives :")
        for elem in self._negatives:
            print ("----------------------")
            elem.affiche_fluent()

    # This is the properties, that protect the use of the _gets and _sets of the class

    positives = property(_get_positives, _set_positives)
    negatives = property(_get_negatives, _set_negatives)

if __name__ == '__main__':

    # Test of the Exp class

    # Creation of a positive Fluent
    positive = Fluents("on", "a", "c", "g")
    # Display of the recently created Fluent

    # Creation of a negative fluent
    negative = Fluents("handempty")

    # Creation of a list of positives and a list of negatives :
    # Creation dof two new positives
    positive2 = Fluents("on", " z", "e")
    positive3 = Fluents("under", "y", "u")
    positives_liste = [positive2, positive3]

    # Creation of two new negatives
    negative2 = Fluents("handempty")
    negative3 = Fluents("on", "k", "l")
    negatives_liste = [negative2, negative3]

    # Creation of the obecjt Exp with the lists
    exp_with_list = Exp(positives_liste, negatives_liste)
    exp_with_list.affiche()


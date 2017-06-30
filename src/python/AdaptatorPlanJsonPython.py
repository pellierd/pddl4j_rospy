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
from Action import Action
from SequentialPlan import SequentialPlan
from Plan import Plan
import json
from pprint import pprint


def unicodes_convert_to_str(unicode_chaine_or_list):
    """
    This function allow us to convert a unicode string or a list of unicode strings into a str or a list of str
    in order to use them to create Fluents
    :param unicode_chaine_or_list: either a list of unicode string or a unicode string that need to be convert into str
    :return: a list of str or a str
    """
    if isinstance(unicode_chaine_or_list, unicode):
        return unicode_chaine_or_list.encode('utf-8')
    elif isinstance(unicode_chaine_or_list, list):
        list_str = list()
        for elem_list in unicode_chaine_or_list:
            if isinstance(elem_list, unicode):
                elem_list = elem_list.encode('utf-8')
                list_str.append(elem_list)
            else:
                raise SyntaxError("Error function byteify in class AdaptatorPlanJsonPython.py,"
                                  " not unicode value in the list provided")
        return list_str
    else:
        raise SyntaxError("error function byteify in class AdaptatorPlanJsonPython.py,"
                          " not a unicode type or a list of unicode type")


def split_chain_str_space(chaine_str):
    """
    This function is designed in order to split the json's string of a fluents and get in a list the predicate
    and the parameters.
    :param chaine_str: the fluents in a format of a single str
    :return: a list with the component of chaine_str, the 1st comoponent is the predicat and the others are
    the parameters
    """
    liste_chain_split = list()
    if isinstance(chaine_str,str):
        liste_chain_split = chaine_str.split()
    else:
        raise SyntaxError("Error in function split_chain_str_space in class AdaptatorPlanJsonPython.py,"
                          " the value of the parameter is not a str")
    return liste_chain_split


def delete_parenthesis_str(chaine_str):
    """
    This function deletes the parenthesis of a string, in our case it remove the parenthesis of the json string that
    define a fluents
    :param chaine_str: the json string that define a fluent
    :return: chaine_str without the parenthesis
    """
    if isinstance(chaine_str,str):
        new_str = chaine_str.replace("(", "")
        new_str_final = new_str.replace(")", "")
        return new_str_final
    else:
        raise SyntaxError("Error in function delete_parentesis_str in class AdaptatorPlanJsonPython.py,"
                          " the value of the parameter is not a str")


def get_precondition(action):
    """
    This function return a Exp object from a action json object
    :param action: action json object
    :return: a Exp
    """
    fluents_positives_list = get_fluents_from_json_object(action["Preconditions"]["Positives"])
    fluents_negatives_list = get_fluents_from_json_object(action["Preconditions"]["Negatives"])

    preconditions = Exp(fluents_positives_list, fluents_negatives_list)

    return preconditions


def get_fluents_from_json_object(json_object):
    """
    This function return a liste of fluents from a json object that contains positives or negatives
    :param json_object: positives or negatives in json object
    :return: a list of fluents
    """
    # We create a list of fluents we want to return
    liste_fluents = list()

    # we convert the json array we have in parameterinto a str array (the json array in in unicode
    liste_str_converted = unicodes_convert_to_str(json_object)

    # This loop will create the fluents and add them into the fluents list we created earlier
    for elem_liste_str_converted in liste_str_converted:

        # We split the chain we get when we iterate on the liste_str_converted in order to create the Fluents
        # We want to extract the predicate and the
        liste_split_str_liste_str_converted = split_chain_str_space(delete_parenthesis_str(elem_liste_str_converted))
        if len(liste_split_str_liste_str_converted) != 0:
            fluent = Fluents(liste_split_str_liste_str_converted[0], liste_split_str_liste_str_converted[1:])
            liste_fluents.append(fluent)

    return liste_fluents


def get_cond_exp_condition(json_object):
    """
    This function return a cond_exp's condition for an action jsonb object
    :param json_object: json condition object
    :return: a cond_exp's condition object
    """

    # We extract the positives & negatives from the json's condition object
    liste_condition_positives = get_fluents_from_json_object(json_object["Condition"]["Positives"])
    liste_condition_negatives = get_fluents_from_json_object(json_object["Condition"]["Negatives"])

    # We create the Exp for the condition
    condition = Exp(liste_condition_positives, liste_condition_negatives)

    return condition


def get_cond_exp_effect(json_object):
    """
    This function return a cond_exp's effect for an action jsonb object
    :param json_object: json effect object
    :return: a cond_exp's effect object
    """
    # We extract the positives & negatives from the json's effect object
    liste_effect_positives = get_fluents_from_json_object(json_object["Effect"]["Positives"])
    liste_effect_negatives = get_fluents_from_json_object(json_object["Effect"]["Negatives"])

    # We create the Exp for the effect
    effect = Exp(liste_effect_positives, liste_effect_negatives)

    return effect


def get_cond_exp(action):
    """
    This function return the list of CondExp from a acton object
    :param action: json action object
    :return: a list of CodnExp
    """

    # This is the list that we want to return
    liste_cond_exp = list()

    # This is the list of the cond_exp in json format
    liste_cond_exp_json = action["Condition_Expressions"]

    # This loop will create a CondExp object and append it to the list of CondExp we want to return in this function
    for elem_liste_cond_exp in liste_cond_exp_json:

        condition = get_cond_exp_condition(elem_liste_cond_exp)
        effect = get_cond_exp_effect(elem_liste_cond_exp)

        cond_exp = CondExp(effect, condition)

        liste_cond_exp.append(cond_exp)

    return liste_cond_exp


def get_sequential_plan(json_object):
    """
    This function return a sequential_plan created from the informations extracted from the json file.
    :param json_object: the json at his root
    :return: sequential_plan
    """

    # On affiche les timeSpecifiers
    pprint(json_object["timeSpecifiers"])

    # On les stocks
    time_specifiers = data["timeSpecifiers"]

    # Liste qui contiendra les objets actions
    actions = list()

    # Liste qui contiendra les objects actions en json pour permettre de creer les objects actions
    # pour la liste actions
    actions_json = list()

    # Boucle permettant de recuperer les objects json actions
    for elem_temps in time_specifiers:
        actions_json.append(data["Action" + " " + repr(elem_temps)])

    # Boucle permettant de creer un object action est de l'ajouter a la liste d'actions
    for elem_action in actions_json:
        # We get the precondition from the json
        precondition = get_precondition(elem_action)

        # We get the list of CondExp from the Json
        list_cond_exp = get_cond_exp(elem_action)

        # We create an action
        action = Action(unicodes_convert_to_str(elem_action["Names"]),
                        unicodes_convert_to_str(elem_action["Parameters"]), precondition, list_cond_exp)

        # We add the action to the list of actions
        actions.append(action)

    plan = SequentialPlan(actions)

    return plan


def load_json_file(name_of_the_json_file):
    """
    This function load the json in an object in order to extract the data from it
    :param name_of_the_json_file: it's the name of the json file generated by the java part of the module,
     it should be a str
    :return: a json object that will be used in order to get information about the plan
    """
    if isinstance(name_of_the_json_file, str):
        # We open the json file and put it in a variable called data
        with open(name_of_the_json_file) as data_file:
            json_data = json.load(data_file)
            return json_data
    else:
        raise SyntaxError("Error in function load_json_file in AdaptatorPlanJsonPython.py,"
                          "the value of the parameter is not a str")


def getSequentialPlanFromJson(name_of_the_json_file):
    global data
    data = load_json_file(name_of_the_json_file)
    if data["Type_de_plan"] == 1:
        print("Sequential Plan :")
        sequential_plan = get_sequential_plan(data)
        return sequential_plan
    else:
        return False

if __name__ == "__main__":

    jsonPath = str(raw_input("json path >> "))

    print ("Debut de la recuperation des donnees : \n")
    data = load_json_file(jsonPath)

    # If the plan from the json file is a Sequential Plan
    #if data["Type_de_plan"] == 1:
        #print ("Sequential Plan :")

        #sequential_plan = get_sequential_plan(data)
    sequential_plan = getSequentialPlanFromJson(jsonPath)
    if sequential_plan != False:
        for action in sequential_plan.actions():
            print("Action name : " + action._get_name())

        # Partie de test
        #pprint(sequential_plan.cost())
        #pprint(sequential_plan.makespan())
        #pprint(sequential_plan.size())

        #pprint(sequential_plan.is_empty())
        #pprint(sequential_plan.is_time_specifiers_out_of_bound(17))
        #liste = sequential_plan.get_actions_set(3)
        #for elem in liste:
        #    elem.affiche()





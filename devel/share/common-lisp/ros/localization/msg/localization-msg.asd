
(cl:in-package :asdf)

(defsystem "localization-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "FloatStamp" :depends-on ("_package_FloatStamp"))
    (:file "_package_FloatStamp" :depends-on ("_package"))
    (:file "Keyop" :depends-on ("_package_Keyop"))
    (:file "_package_Keyop" :depends-on ("_package"))
  ))
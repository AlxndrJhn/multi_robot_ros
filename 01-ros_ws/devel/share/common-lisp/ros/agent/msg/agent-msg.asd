
(cl:in-package :asdf)

(defsystem "agent-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "bid" :depends-on ("_package_bid"))
    (:file "_package_bid" :depends-on ("_package"))
  ))
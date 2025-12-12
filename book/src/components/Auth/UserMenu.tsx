/**
 * User Menu component with avatar and dropdown options
 */
import React, { useState, useRef, useEffect } from "react";
import { useAuth } from "../ChatWidget/hooks/useAuth";
import { SignOutButton } from "./SignOutButton";
import styles from "./Auth.module.css";

interface UserMenuProps {
  /** Optional callback after sign-out */
  onSignOut?: () => void;
}

/**
 * Displays user avatar with dropdown menu for account actions
 */
export function UserMenu({ onSignOut }: UserMenuProps) {
  const { user, isAuthenticated } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const menuRef = useRef<HTMLDivElement>(null);

  // Close menu when clicking outside
  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    }

    document.addEventListener("mousedown", handleClickOutside);
    return () => document.removeEventListener("mousedown", handleClickOutside);
  }, []);

  // Close menu on escape key
  useEffect(() => {
    function handleEscape(event: KeyboardEvent) {
      if (event.key === "Escape") {
        setIsOpen(false);
      }
    }

    document.addEventListener("keydown", handleEscape);
    return () => document.removeEventListener("keydown", handleEscape);
  }, []);

  if (!isAuthenticated || !user) {
    return null;
  }

  const initials = user.name
    ? user.name
        .split(" ")
        .map((n) => n[0])
        .join("")
        .toUpperCase()
        .slice(0, 2)
    : user.email?.charAt(0).toUpperCase() || "?";

  return (
    <div className={styles.userMenu} ref={menuRef}>
      <button
        className={styles.userMenuTrigger}
        onClick={() => setIsOpen(!isOpen)}
        aria-expanded={isOpen}
        aria-haspopup="true"
        type="button"
      >
        <div className={styles.userAvatar}>
          {user.image ? (
            <img src={user.image} alt={user.name || "User"} />
          ) : (
            <span className={styles.userInitials}>{initials}</span>
          )}
        </div>
      </button>

      {isOpen && (
        <div className={styles.userMenuDropdown} role="menu">
          <div className={styles.userMenuHeader}>
            <span className={styles.userName}>{user.name}</span>
            <span className={styles.userEmail}>{user.email}</span>
          </div>
          <div className={styles.userMenuDivider} />
          <div className={styles.userMenuItem}>
            <SignOutButton
              variant="text"
              onSignOut={() => {
                setIsOpen(false);
                onSignOut?.();
              }}
            />
          </div>
        </div>
      )}
    </div>
  );
}

export default UserMenu;

/**
 * User Avatar component with Google image or fallback initials
 */
import React from "react";
import styles from "./Auth.module.css";

interface UserAvatarProps {
  /** User's display name */
  name?: string;
  /** User's email (fallback for initials) */
  email?: string;
  /** URL to user's profile image */
  image?: string | null;
  /** Size of the avatar */
  size?: "small" | "medium" | "large";
  /** Optional click handler */
  onClick?: () => void;
}

/**
 * Displays user avatar with Google profile image or fallback initials
 */
export function UserAvatar({
  name,
  email,
  image,
  size = "medium",
  onClick,
}: UserAvatarProps) {
  // Generate initials from name or email
  const initials = getInitials(name, email);

  const sizeClass =
    size === "small" ? styles.avatarSmall :
    size === "large" ? styles.avatarLarge :
    styles.userAvatar;

  const Component = onClick ? "button" : "div";
  const props = onClick ? { onClick, type: "button" as const } : {};

  return (
    <Component className={sizeClass} {...props}>
      {image ? (
        <img src={image} alt={name || "User"} />
      ) : (
        <span className={styles.userInitials}>{initials}</span>
      )}
    </Component>
  );
}

/**
 * Generate initials from name or email
 */
function getInitials(name?: string, email?: string): string {
  if (name) {
    return name
      .split(" ")
      .map((n) => n[0])
      .join("")
      .toUpperCase()
      .slice(0, 2);
  }

  if (email) {
    return email.charAt(0).toUpperCase();
  }

  return "?";
}

export default UserAvatar;

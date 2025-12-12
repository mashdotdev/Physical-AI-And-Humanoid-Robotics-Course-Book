/**
 * User Profile component showing name and email
 */
import React from "react";
import { UserAvatar } from "./UserAvatar";
import styles from "./Auth.module.css";

interface UserProfileProps {
  /** User's display name */
  name: string;
  /** User's email address */
  email: string;
  /** URL to user's profile image */
  image?: string | null;
  /** Show compact version */
  compact?: boolean;
}

/**
 * Displays user profile information with avatar
 */
export function UserProfile({
  name,
  email,
  image,
  compact = false,
}: UserProfileProps) {
  if (compact) {
    return (
      <div className={styles.userProfileCompact}>
        <UserAvatar name={name} email={email} image={image} size="small" />
        <span className={styles.userProfileName}>{name}</span>
      </div>
    );
  }

  return (
    <div className={styles.userProfile}>
      <UserAvatar name={name} email={email} image={image} size="large" />
      <div className={styles.userProfileInfo}>
        <span className={styles.userProfileName}>{name}</span>
        <span className={styles.userProfileEmail}>{email}</span>
      </div>
    </div>
  );
}

export default UserProfile;
